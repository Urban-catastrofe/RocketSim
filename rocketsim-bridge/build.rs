fn main() {
    cxx_build::bridge("src/lib.rs")
        .std("c++20")
        .compile("rocketsim_bridge");
    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=src/arena_wrapper.rs");
    println!("cargo:rerun-if-changed=src/conversions.rs");
}
