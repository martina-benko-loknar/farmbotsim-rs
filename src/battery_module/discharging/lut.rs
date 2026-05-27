use std::fs::File;
use std::io::BufReader;

#[derive(Debug, Clone, PartialEq)]
pub struct VoltageDropLUT {
    pub entries: Vec<LUTEntry>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct LUTEntry {
    pub slope_rad: f32,
    pub speed_mps: f32,
    pub voltage_drop_per_m: f32,
}

impl VoltageDropLUT {
    pub fn from_csv(path: &str) -> Self {
        let file = File::open(path)
            .expect("Cannot open LUT file");

        let mut rdr = csv::Reader::from_reader(BufReader::new(file));

        let mut entries = Vec::new();

        for result in rdr.records() {
            let record = result.expect("Invalid CSV row");

            let slope_rad: f32 = record[0].parse().unwrap();
            let speed_mps: f32 = record[1].parse().unwrap();
            let vdrop: f32 = record[2].parse().unwrap();

            entries.push(LUTEntry {
                slope_rad,
                speed_mps,
                voltage_drop_per_m: vdrop,
            });
        }

        Self { entries }
    }

    pub fn get(&self, slope: f32, speed: f32) -> f32 {
        let mut best = None;
        let mut best_dist = f32::INFINITY;

        for e in &self.entries {
            let ds = e.slope_rad - slope;
            let dv = e.speed_mps - speed;

            let dist = ds * ds + dv * dv;

            if dist < best_dist {
                best_dist = dist;
                best = Some(e.voltage_drop_per_m);
            }
        }

        best.unwrap_or(0.0)
    }
}