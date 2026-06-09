use std::{
    fs::File,
    io::{BufRead, BufReader},
};

use crate::terrain::height_map::HeightMap;

#[derive(Clone, Debug)]
struct RawPoint {
    lat: f64,
    lon: f64,
    alt: f64,
}

pub struct TerrainLoader;

impl TerrainLoader {
    pub fn from_gps_csv(path: &str) -> HeightMap {
        let points = read_csv(path);

        assert!(!points.is_empty(), "Empty terrain file");

        // --------------------------------------------------
        // 1. GPS origin (SW corner)
        // --------------------------------------------------
        let min_lat = points.iter().map(|p| p.lat).fold(f64::INFINITY, f64::min);
        let min_lon = points.iter().map(|p| p.lon).fold(f64::INFINITY, f64::min);

        const R: f64 = 6_371_000.0;

        fn lat_to_y(lat: f64, lat0: f64) -> f64 {
            (lat - lat0).to_radians() * R
        }

        fn lon_to_x(lon: f64, lon0: f64, lat0: f64) -> f64 {
            (lon - lon0).to_radians() * R * lat0.to_radians().cos()
        }

        // --------------------------------------------------
        // 2. Convert to meters
        // --------------------------------------------------
        let mut xs: Vec<f64> = Vec::with_capacity(points.len());
        let mut ys: Vec<f64> = Vec::with_capacity(points.len());

        for p in &points {
            xs.push(lon_to_x(p.lon, min_lon, min_lat));
            ys.push(lat_to_y(p.lat, min_lat));
        }

        // --------------------------------------------------
        // 3. Sort copies for spacing detection
        // --------------------------------------------------
        let mut xs_sorted = xs.clone();
        let mut ys_sorted = ys.clone();

        xs_sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        ys_sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());

        // --------------------------------------------------
        // 4. Find first meaningful spacing in x and y
        // --------------------------------------------------
        let cell_width_x = first_nonzero_diff(&xs_sorted);
        let cell_width_y = first_nonzero_diff(&ys_sorted);

        //println!("cell width x: {}", cell_width_x);
        //println!("cell width y: {}", cell_width_y);

        assert!(cell_width_x > 0.0, "Invalid X spacing detected");
        assert!(cell_width_y > 0.0, "Invalid Y spacing detected");

        // --------------------------------------------------
        // 5. Compute grid bounds
        // --------------------------------------------------
        let min_x = xs.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_x = xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        let min_y = ys.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_y = ys.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        let width = ((max_x - min_x) / cell_width_x).round() as usize + 1;
        let height = ((max_y - min_y) / cell_width_y).round() as usize + 1;

        // --------------------------------------------------
        // 6. Build grid
        // --------------------------------------------------
        let mut grid = vec![vec![0.0_f64; width]; height];

        for (i, p) in points.iter().enumerate() {
            let x = xs[i] - min_x;
            let y = ys[i] - min_y;

            let gx = (x / cell_width_x).round() as usize;
            let gy = (y / cell_width_y).round() as usize;

            if gx < width && gy < height {
                grid[gy][gx] = p.alt;
            }
        }

        // --------------------------------------------------
        // 7. Return height map
        // --------------------------------------------------
        HeightMap {
            width,
            height,
            cell_size_x_m: cell_width_x,
            cell_size_y_m: cell_width_y,
            elevations: grid,
            origin_x: min_x,
            origin_y: min_y,
        }
    }
}

/// ------------------------------------------------------
/// Find first meaningful spacing between distinct values
/// ------------------------------------------------------
fn first_nonzero_diff(values: &[f64]) -> f64 {
    for w in values.windows(2) {
        let d = (w[1] - w[0]).abs();
        if d > 1e-6 {
            return d;
        }
    }
    0.0
}

/// ------------------------------------------------------
/// CSV reader
/// ------------------------------------------------------
fn read_csv(path: &str) -> Vec<RawPoint> {
    let file = File::open(path).expect("Cannot open terrain file");
    let reader = BufReader::new(file);

    let mut points = Vec::new();

    for line in reader.lines().skip(1).flatten() {
        let parts: Vec<&str> = line.split(',').collect();
        if parts.len() < 3 {
            continue;
        }

        let lat = parts[0].trim().parse::<f64>().ok();
        let lon = parts[1].trim().parse::<f64>().ok();
        let alt = parts[2].trim().parse::<f64>().ok();

        if let (Some(lat), Some(lon), Some(alt)) = (lat, lon, alt) {
            points.push(RawPoint { lat, lon, alt });
        }
    }

    points
}
// use std::{
//     fs::File,
//     io::{BufRead, BufReader},
// };

// use crate::terrain::height_map::HeightMap;

// #[derive(Clone, Debug)]
// struct RawPoint {
//     lat: f64,
//     lon: f64,
//     alt: f32,
// }

// pub struct TerrainLoader;

// impl TerrainLoader {
//     pub fn from_gps_csv(path: &str, resolution_m: f32) -> HeightMap {
//         let points = read_csv(path);

//         assert!(
//             !points.is_empty(),
//             "Terrain CSV contains no valid points"
//         );

//         // -----------------------------
//         // 1. Find GPS origin (min corner)
//         // -----------------------------
//         let min_lat = points
//             .iter()
//             .map(|p| p.lat)
//             .fold(f64::INFINITY, f64::min);

//         let min_lon = points
//             .iter()
//             .map(|p| p.lon)
//             .fold(f64::INFINITY, f64::min);

//         // -----------------------------
//         // 2. GPS → local meters
//         // -----------------------------
//         const R: f64 = 6_371_000.0;

//         fn lat_to_y(lat: f64, lat0: f64) -> f64 {
//             (lat - lat0).to_radians() * R
//         }

//         fn lon_to_x(lon: f64, lon0: f64, lat0: f64) -> f64 {
//             (lon - lon0).to_radians() * R * lat0.to_radians().cos()
//         }

//         let mut xs = Vec::with_capacity(points.len());
//         let mut ys = Vec::with_capacity(points.len());

//         for p in &points {
//             xs.push(lon_to_x(p.lon, min_lon, min_lat));
//             ys.push(lat_to_y(p.lat, min_lat));
//         }

//         // -----------------------------
//         // 3. Grid bounds
//         // -----------------------------
//         let min_x = xs.iter().cloned().fold(f64::INFINITY, f64::min);
//         let max_x = xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

//         let min_y = ys.iter().cloned().fold(f64::INFINITY, f64::min);
//         let max_y = ys.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

//         let width = ((max_x - min_x) / resolution_m as f64).ceil() as usize + 1;
//         let height = ((max_y - min_y) / resolution_m as f64).ceil() as usize + 1;

//         // -----------------------------
//         // 4. Build elevation grid
//         // -----------------------------
//         let mut grid = vec![vec![0.0_f32; width]; height];

//         for (i, p) in points.iter().enumerate() {
//             let x = xs[i] - min_x;
//             let y = ys[i] - min_y;

//             let gx = (x / resolution_m as f64) as usize;
//             let gy = (y / resolution_m as f64) as usize;

//             if gx < width && gy < height {
//                 grid[gy][gx] = p.alt;
//             }
//         }

//         // -----------------------------
//         // 5. Return height map
//         // -----------------------------
//         HeightMap {
//             width,
//             height,
//             resolution_m,
//             elevations: grid,

//             // IMPORTANT:
//             // now origin is in LOCAL METERS (not GPS anymore)
//             origin_x: min_x,
//             origin_y: min_y,
//         }
//     }
// }

// /// Reads raw CSV points
// fn read_csv(path: &str) -> Vec<RawPoint> {
//     let file = File::open(path).expect("Cannot open terrain file");
//     let reader = BufReader::new(file);

//     let mut points = Vec::new();

//     for line in reader.lines().skip(1).flatten() {
//         let parts: Vec<&str> = line.split(',').collect();
//         if parts.len() < 3 {
//             continue;
//         }

//         let lat = match parts[0].trim().parse::<f64>() {
//             Ok(v) => v,
//             Err(_) => continue,
//         };

//         let lon = match parts[1].trim().parse::<f64>() {
//             Ok(v) => v,
//             Err(_) => continue,
//         };

//         let alt = match parts[2].trim().parse::<f32>() {
//             Ok(v) => v,
//             Err(_) => continue,
//         };

//         points.push(RawPoint { lat, lon, alt });
//     }

//     points
// }