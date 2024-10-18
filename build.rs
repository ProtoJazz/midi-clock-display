use std::env;
use std::path::PathBuf;
fn main() {
    embuild::espidf::sysenv::output();
    let partitions_csv = PathBuf::from("partitions.csv");

    // Instruct CMake to use the custom partition table
    println!("cargo:rustc-link-search={}", partitions_csv.display());

    // Re-run the build if partitions.csv is changed
    println!("cargo:rerun-if-changed=partitions.csv");
}
