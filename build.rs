extern crate cmake;
use cmake::Config;

fn main()
{
    // compile engine
    let dst = Config::new("src").build();       
    println!("cargo:rustc-link-search=native={}", dst.display());
    
    // link with engine
    println!("cargo:rustc-link-lib=static=engine");
    // println!("cargo:rustc-link-lib=static=engine-debug");
    
    // backend dependences
    println!("cargo:rustc-link-lib=dylib=stdc++");
    println!("cargo:rustc-link-lib=dylib=keystone");
    println!("cargo:rustc-link-lib=dylib=capstone");
    println!("cargo:rustc-link-lib=dylib=yaml-cpp");
    
    // create static version
    // println!("cargo:rustc-link-lib=static=keystone");
    // println!("cargo:rustc-link-lib=static=capstone");
    // println!("cargo:rustc-link-lib=static=yaml-cpp");
    // println!("cargo:rustc-link-lib=static=stdc++");
    // println!("cargo:rustc-link-lib=static=gcc");
    // println!("cargo:rustc-link-arg=-static-libgcc");
    
}
