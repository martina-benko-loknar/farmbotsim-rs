use egui::Vec2;
use rand::{Rng, SeedableRng};
use rand::rngs::StdRng;
use std::f32::consts::PI;

use crate::units::angle::{Angle, AngleUnit};

/// Get random Vec2 with lenght 1.
pub fn random_vec2() -> Vec2 {
    //let mut rng = rand::rng(); // Random number generator
    let mut rng = StdRng::seed_from_u64(42);

    // Generate a random angle in radians between 0 and 2π
    let angle = rng.random_range(0.0..2.0 * PI);

    // Convert the angle to a direction (unit vector)
    let x = angle.cos();
    let y = angle.sin();

    Vec2::new(x, y)
}

pub trait Vec2Rotate {
    /// Rotates Vec2 by a given angle
    fn rotate(&self, angle: Angle) -> Vec2;
}

impl Vec2Rotate for Vec2 {
    fn rotate(&self, angle: Angle) -> Vec2 {
        match angle.unit {
            AngleUnit::Radians => {
                rotate_radians(*self, angle.value)
            },
            AngleUnit::Degrees => {
                rotate_radians(*self, angle.value.to_radians())
            }
        }
    }
}


pub trait ExtendedVec2 {
    /// Format Vec2 into `String` with decimals.
    fn fmt(&self, n_decimals: usize) -> String;
}

impl ExtendedVec2 for Vec2 {
    fn fmt(&self, n_decimals: usize) -> String {
        format!("({:.*}, {:.*})", n_decimals, self.x, n_decimals, self.y)
    }
}

/// Rotates Vec2 by angle in radians.
fn rotate_radians(vec2: Vec2, angle_rad: f32) -> Vec2 {
    let cos = angle_rad.cos();
    let sin = angle_rad.sin();
    Vec2::new(
        vec2.x * cos - vec2.y * sin,
        vec2.x * sin + vec2.y * cos,
    )
}