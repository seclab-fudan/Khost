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
}
