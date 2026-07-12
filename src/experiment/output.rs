use std::error::Error;

pub fn create_results_subdir(
    base_output: &str,
    subdir: &str,
) -> Result<String, Box<dyn Error>> {
    let output_dir = format!("{base_output}/{subdir}");

    std::fs::create_dir_all(&output_dir)?;

    Ok(output_dir)
}
