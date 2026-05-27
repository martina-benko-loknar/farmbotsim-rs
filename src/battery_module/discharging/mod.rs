pub mod traits;
pub mod analytical;
pub mod empirical;
pub mod lut;
pub mod physics_model;

pub use traits::ConsumptionModel;
pub use analytical::AnalyticalConsumptionModel;
pub use empirical::EmpiricalConsumptionModel;
pub use lut::VoltageDropLUT;