use std::process::Command;

pub fn run_viz_script(
    script: &str,
    input_path: &str,
    output_dir: &str,
) {
    let status = Command::new("python")
        .arg(script)
        .arg(input_path)
        .arg(output_dir)
        .status();

    match status {
        Ok(s) if s.success() => {}
        Ok(_) => {
            eprintln!("Visualization script failed: {script}");
        }
        Err(e) => {
            eprintln!("Failed to run {script}: {e}");
        }
    }
}