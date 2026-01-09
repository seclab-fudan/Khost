use clap::{App, Arg};
use std::{env, path::PathBuf};
use std::{ffi::CString, fs::metadata, path::Path, time::Duration};

use libafl::{
    bolts::{
        current_nanos,
        launcher::Launcher,
        os::Cores,
        rands::StdRand,
        shmem::{ShMem, ShMemProvider, StdShMemProvider},
        tuples::{tuple_list, Merge},
        AsSlice, AsMutSlice,
    },
    corpus::{Corpus, OnDiskCorpus},
    events::EventConfig,
    executors::{inprocess::InProcessExecutor, ExitKind, TimeoutExecutor},
    feedback_or, feedback_and_fast,
    feedbacks::{CrashFeedback, MapFeedbackState, MaxMapFeedback, TimeFeedback},
    fuzzer::{Fuzzer, StdFuzzer},
    inputs::{BytesInput, HasTargetBytes},
    monitors::MultiMonitor,
    mutators::{
        scheduled::{havoc_mutations, tokens_mutations, StdScheduledMutator},
        Tokens,
    },
    observers::{ConstMapObserver, HitcountsMapObserver, TimeObserver},
    schedulers::{IndexesLenTimeMinimizerScheduler, QueueScheduler},
    stages::mutational::StdMutationalStage,
    state::{HasCorpus, HasMetadata, StdState},
    Error,
};

// #[link(name = "engine-debug", kind = "static")]
#[link(name = "engine", kind = "static")]
extern "C" {
    fn init_execution(config_path: *const u8, working_path: *const u8) -> bool;
    fn reset_execution(input: *const u8, length: usize) -> bool;
    fn start_execution(timeout: i32) -> i32;
}

pub fn main() {
    let options = App::new("KVM for MCU")
        .version("1.0.0")
        .about("Speed up MCU firmware rehosting with KVM native running.")
        .arg(
            Arg::with_name("config")
                .short("b")
                .long("config")
                .value_name("CONFIG")
                .help("Sets path config")
                .required(true)
                .takes_value(true),
        )
        .arg(
            Arg::with_name("input")
                .short("i")
                .long("input_file")
                .value_name("INPUT")
                .help("Specifies path to input file/directory")
                .required(false)
                .takes_value(true),
        )
        .arg(
            Arg::with_name("dir")
                .short("d")
                .long("working_dir")
                .value_name("DIR")
                .help("working dir of the fuzzer")
                .required(false)
                .takes_value(true),
        )
        .arg(
            Arg::with_name("fuzz")
                .short("f")
                .long("fuzz")
                .help("Starts as fuzzer")
                .required(false),
        )
        .arg(
            Arg::with_name("token-file")
                .short("t")
                .long("token-file")
                .value_name("TOKENS")
                .help("Specifies path to file with token dictionary")
                .required(false)
                .takes_value(true),
        )
        .arg(
            Arg::with_name("cores")
                .short("c")
                .long("cores")
                .value_name("CORES")
                .help("Specifies the CPU IDs used for fuzzing, e.g., '1,2-4,6' or 'all'")
                .required(false)
                .takes_value(true),
        )
        .arg(
            Arg::with_name("broker-port")
                .short("p")
                .long("broker-port")
                .value_name("PORT")
                .help("Specifies the port the LibAFL broker is running on (default: 1337)")
                .required(false)
                .takes_value(true),
        )
        .get_matches();

    let broker_port: u16 = if options.is_present("broker-port") {
        options
            .value_of("broker-port")
            .unwrap()
            .parse::<u16>()
            .unwrap()
    } else {
        1337
    };

    let cores: Cores = if options.is_present("cores") {
        println!("{:?}", options.value_of("cores"));
        Cores::from_cmdline(options.value_of("cores").unwrap()).unwrap()
    } else {
        Cores::from(vec![2_usize, 3_usize])
    };

    // fuzzware model path of the firmware
    let config_path = options.value_of("config").unwrap();
    assert_eq!(Path::new(config_path).is_file(), true);
    let config_path = CString::new(config_path).expect("Failed to get config path");

    // set working dir
    let current_dir = env::current_dir().unwrap();
    let working_dir = if options.is_present("dir") {
        options.value_of("dir").unwrap()
    } else {
        current_dir.to_str().unwrap()
    };
    let working_path = CString::new(working_dir).expect("Failed to get working path");
    assert_eq!(Path::new(working_dir).is_dir(), true);

    // get input dir
    let input_opt = options.value_of("input");
    let default_dir = PathBuf::from(working_dir).join(PathBuf::from("./corpus"));
    let input_dir: &str;
    if let Some(input_val) = input_opt {
        input_dir = input_val;
    } else {
        input_dir = default_dir.to_str().unwrap();
    }

    // The default, OS-specific privider for shared memory
    let global_shmem_provider = StdShMemProvider::new().unwrap();

    // output monitor
    let monitor = MultiMonitor::new(|s| println!("{}", s));

    let mut run_client = |state: Option<StdState<_, _, _, _, _>>,
                        mut restarting_mgr,
                        _size: usize| {
        // check input directory
        let md = metadata(input_dir).expect("Failed to read specified Input directory");
        if !md.is_dir() {
            panic!("Specified Input Option does not seem to be a directory")
        }

        let corpus_dirs = &[PathBuf::from(input_dir)];
        let objective_dir = PathBuf::from(working_dir).join(PathBuf::from("./crashes"));
        let queue_dir = PathBuf::from(working_dir).join(PathBuf::from("./queue"));

        const MAP_SIZE: usize = 1 << 16;

        // The default, OS-specific privider for shared memory
        let mut shmem_provider = StdShMemProvider::new().unwrap();

        // The coverage map shared between observer and executor
        let shmem = shmem_provider.new_shmem(MAP_SIZE).unwrap();
        let shmem = Box::leak(Box::new(shmem)); 
        // let the forkserver know the shmid
        shmem.write_to_env("__AFL_SHM_ID").unwrap();
        let shmem_buf = shmem.as_mut_slice();

        // init engine
        unsafe {
            if !init_execution(
                config_path.as_bytes_with_nul().as_ptr(),
                working_path.as_bytes_with_nul().as_ptr()
            ) {
                panic!("Failed to init board");
            }
        }
        println!("[*] INIT BOARD FINISH");

        // Create an observation channel using the signals map
        let edges_observer = HitcountsMapObserver::new(
            ConstMapObserver::<_, MAP_SIZE>::new("shared_mem", shmem_buf)
        );

        // Create an observation channel to keep track of the execution time
        let time_observer = TimeObserver::new("time");

        // The state of the edges feedback.
        let feedback_state = MapFeedbackState::with_observer(&edges_observer);

        // Feedback to rate the interestingness of an input
        // This one is composed by two Feedbacks in OR
        let feedback = feedback_or!(
            // New maximization map feedback linked to the edges observer and the feedback state
            MaxMapFeedback::new_tracking(&feedback_state, &edges_observer, true, false),
            // Time feedback, this one does not need a feedback state
            TimeFeedback::new_with_observer(&time_observer)
        );

        // The state of the edges feedback for crashes.
        let objective_state = MapFeedbackState::new("crash_edges", 1 << 16);

        // A feedback to choose if an input is a solution or not
        // We want to do the same crash deduplication that AFL does
        let objective = feedback_and_fast!(
            // Must be a crash
            CrashFeedback::new(),
            // Take it onlt if trigger new coverage over crashes
            MaxMapFeedback::new(&objective_state, &edges_observer)
        );

        // create a State from scratch
        let mut state = state.unwrap_or_else(|| {
            StdState::new(
                StdRand::with_seed(current_nanos()),
                OnDiskCorpus::new(queue_dir).unwrap(),
                OnDiskCorpus::new(objective_dir).unwrap(),
                tuple_list!(feedback_state, objective_state),
            )
        });

        // A queue policy to get testcasess from the corpus and minimize the time
        let scheduler = IndexesLenTimeMinimizerScheduler::new(QueueScheduler::new());

        // A fuzzer with feedbacks and a corpus scheduler
        let mut fuzzer = StdFuzzer::new(scheduler, feedback, objective);

        // The wrapped harness function, calling out to the LLVM-style harness
        let mut harness = |input: &BytesInput| {
            let target = input.target_bytes();
            let buf = target.as_slice();
            unsafe {
                if !reset_execution(buf.as_ptr(), buf.len()) {
                    return ExitKind::Crash;
                }
                let ret = start_execution(10 * 1000);
                match ret {
                    0 => { ExitKind::Ok },
                    1 => { ExitKind::Crash },
                    2 => { ExitKind::Crash },
                    3 => { ExitKind::Timeout},
                    _ => { ExitKind::Crash },
                }
            }
        };

        if options.is_present("token-file") && state.metadata().get::<Tokens>().is_none() {
            println!("[+] Adding tokens to mutator dictionary");
            state.add_metadata(Tokens::from_file(std::path::Path::new(
                options.value_of("token-file").unwrap(),
            ))?);
        }

        // Setup a basic mutator with a mutational stage
        let mutator = StdScheduledMutator::new(havoc_mutations().merge(tokens_mutations()));
        let mut stages = tuple_list!(StdMutationalStage::new(mutator));

        // Create the executor for an in-process function with one observer for edge coverage and one for the execution time
        let mut executor = TimeoutExecutor::new(
            InProcessExecutor::new(
                &mut harness,
                tuple_list!(edges_observer, time_observer),
                &mut fuzzer,
                &mut state,
                &mut restarting_mgr,
            )?,
            Duration::new(10000, 0),
        );

        println!("[*] STARTING FUZZER");

        // In case the corpus is empty (on first run), reset
        if state.corpus().count() < 1 {
            state
                .load_initial_inputs_forced(
                    &mut fuzzer,
                    &mut executor,
                    &mut restarting_mgr,
                    corpus_dirs,
                )
                .unwrap_or_else(|_| panic!("Failed to load initial corpus at {:?}", corpus_dirs));
            println!("We imported {} inputs from disk.", state.corpus().count());
        }
        println!("[*] CROUPS LOAD");

        println!("[*] FUZZER START");
        fuzzer.fuzz_loop(&mut stages, &mut executor, &mut state, &mut restarting_mgr)?;
        Ok(())
    };

    println!("[*] BUILDING FUZZER");
    match Launcher::builder()
        .shmem_provider(global_shmem_provider)
        .configuration(EventConfig::from_name("default"))
        .monitor(monitor)
        .run_client(&mut run_client)
        .cores(&cores)
        .broker_port(broker_port)
        //.stdout_file(Some("/dev/null"))
        .build()
        .launch()
    {
        Ok(()) => (),
        Err(Error::ShuttingDown) => println!("Fuzzing stopped by user. Good bye."),
        Err(err) => panic!("Failed to run launcher: {:?}", err),
    }
}
