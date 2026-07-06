use std::{collections::{HashMap}, fs::File, io::{BufRead, BufReader}, path::Path};

use crate::{
    battery_module::{
        battery_error::BatteryError,
        battery_config::BatteryConfig},
        logger::log_error_and_panic, 
        units::{
            duration::Duration, 
            energy::Energy}, 
        utilities::utils::linear_interpolate};
use crate::cfg::BATTERIES_PATH;

#[derive(Clone, Debug, PartialEq)]
pub struct SeasonalBatteryModel {
    pub jan_max_data: Vec<(u32, f32)>,
    pub jan_min_data: Vec<(u32, f32)>,
    pub jun_max_data: Vec<(u32, f32)>,

    pub start_index: HashMap<String, usize>,
}

impl SeasonalBatteryModel{

    pub fn compute_charge(
        &mut self,
        energy: Energy,
        duration: Duration,
        month: u32,
    ) -> Result<Energy, BatteryError> {
        let new_energy_wh = self.get_morph_x_y(
            energy.to_watt_hour(),
            month,
            duration.to_base_unit() as u32,
        )?.1;

        Ok(Energy::watt_hours(new_energy_wh))
    }

    fn get_month_data_points<P: AsRef<Path>>(file_path: P) -> Vec<(u32, f32)> {
        let path_ref = file_path.as_ref();
        let file = File::open(path_ref).unwrap_or_else(|e| {
            let msg = format!("Failed to open file {:?}: {}", path_ref, e);
            log_error_and_panic(&msg)
        });
        let reader = BufReader::new(file);
        let mut points = Vec::new();

        for line in reader.lines().skip(1).map_while(Result::ok) {
            let parts: Vec<&str> = line.split_whitespace().collect();
            if let (Some(t), Some(v)) = (parts.first(), parts.last()) {
                if let (Ok(t), Ok(v)) = (t.parse::<f32>(), v.parse::<f32>()) {
                    // Time (s) - Energy (Wh)
                    points.push(((t * 3600.0) as u32, v));
                }
            }
        }
        points
        
    }

    // Finds interpolated energy output for a given input time in the month’s dataset.
    fn find_y_for_x_month(&mut self, month: &str, x: u32) -> Result<f32, BatteryError> {
        let data = match month {
            "jan" => &self.jan_min_data,
            "jun" => &self.jun_max_data,
            _ => return Err(BatteryError::UnsupportedMonth(month.to_string())),
        };

        let start = *self.start_index.get(month).unwrap_or(&1);
        for i in start..data.len() {
            let (x0, y0) = data[i - 1];
            let (x1, y1) = data[i];
            if x0 <= x && x <= x1 {
                self.start_index.insert(month.to_string(), std::cmp::max(1, i - 1));
                return Ok(linear_interpolate(x0 as f32, y0, x1 as f32, y1, x as f32));
            }
        }
        Err(BatteryError::NoYForX(x.to_string()))
    }

    // Finds interpolated time needed to reach a given energy in the month’s dataset.
    fn find_x_for_y_month(&mut self, month: &str, y: f32) -> Result<u32, BatteryError> {
        let data = match month {
            "jan" => &self.jan_min_data,
            "jun" => &self.jun_max_data,
            _ => return  Err(BatteryError::UnsupportedMonth(month.to_string())),
        };

        let start = *self.start_index.get(month).unwrap_or(&1);
        for i in start..data.len() {
            let (x0, y0) = data[i - 1];
            let (x1, y1) = data[i];
            if y0 <= y && y <= y1 {
                self.start_index.insert(month.to_string(), std::cmp::max(1, i - 1));
                return Ok(linear_interpolate(y0, x0 as f32, y1, x1 as f32, y) as u32);
            }
        }
        Err(BatteryError::NoXForY(y.to_string()))
    }

    // Calculates morphing time and energy for seasonal solar models.
    pub fn get_morph_x_y(&mut self, y: f32, month: u32, time: u32) -> Result<(u32, f32), BatteryError> {
        let jan_time = self.find_x_for_y_month("jan", y)?;
        let jun_time = self.find_x_for_y_month("jun", y)?;

        let jan_next_time = jan_time + time;
        let jun_next_time = jun_time + time;

        let jan_new_wh = self.find_y_for_x_month("jan", jan_next_time)?;
        let jun_new_wh = self.find_y_for_x_month("jun", jun_next_time)?;

        let month = month as f32;
        let weight1 = (1.0 + (std::f32::consts::PI * (month - 1.0) / 6.0).cos()) / 2.0;
        let weight2 = 1.0 - weight1;

        let new_time = weight1 * jan_next_time as f32 + weight2 * jun_next_time as f32;
        let new_wh = weight1 * jan_new_wh + weight2 * jun_new_wh;
        Ok((new_time as u32, new_wh))
    }


    pub fn from_config(
        config: &BatteryConfig,
    ) -> Self {

        let path =
            format!("{}/{}/",
                BATTERIES_PATH,
                config.name,
            );

        Self {
            jan_max_data:
                Self::get_month_data_points(
                    format!("{}{}", path, config.jan_max)
                ),

            jan_min_data:
                Self::get_month_data_points(
                    format!("{}{}", path, config.jan_min)
                ),

            jun_max_data:
                Self::get_month_data_points(
                    format!("{}{}", path, config.jun_max)
                ),

            start_index: HashMap::from([
                ("jan".to_string(), 1),
                ("jun".to_string(), 1),
            ]),
        }
    }

}
