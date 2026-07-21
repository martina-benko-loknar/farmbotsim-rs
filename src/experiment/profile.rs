use crate::{agent_module::agent_config::AgentConfig, battery_module::battery_config::BatteryConfig};

/// Which experiment "story" to run.
///
/// Field-set and battery model are paired, not independent axes: `Legacy`
/// reproduces the original paper's fields with the simple/seasonal-solar
/// battery model, `Vineyard` pairs the vineyard fields with the
/// CC-CV/physics model built for the Leo Rover.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExperimentProfile {
    Legacy,
    Vineyard,
}

impl ExperimentProfile {
    /// Agent config selecting this profile's battery (capacity/voltage and
    /// charging/discharging model).
    pub fn agent_config_path(&self) -> &'static str {
        match self {
            Self::Legacy => "configs/agent_configs/default.json",
            Self::Vineyard => "configs/agent_configs/leo_rover.json",
        }
    }

    /// Battery config referenced by `agent_config_path`, loaded from disk so
    /// capacity/voltage can't drift out of sync with the battery this
    /// profile actually simulates.
    fn battery_config(&self) -> BatteryConfig {
        let agent_config = AgentConfig::from_json_file(self.agent_config_path());
        BatteryConfig::from_json_file(agent_config.battery)
    }

    pub fn battery_capacity_wh(&self) -> f32 {
        self.battery_config().capacity.to_watt_hour()
    }

    pub fn battery_voltage_v(&self) -> f32 {
        self.battery_config().voltage.to_base_unit()
    }

    /// Field configs to sweep over for this profile.
    pub fn field_configs(&self) -> &'static [&'static str] {
        match self {
            Self::Legacy => &[
                "configs/field_configs/legacy/field1.json",
                "configs/field_configs/legacy/field2.json",
                "configs/field_configs/legacy/field3.json",
                "configs/field_configs/legacy/field4.json",
            ],
            Self::Vineyard => &[
                "configs/field_configs/vineyard/small.json",
                "configs/field_configs/vineyard/medium.json",
                "configs/field_configs/vineyard/large.json",
                "configs/field_configs/vineyard/xlarge.json",
            ],
        }
    }

    /// Field config used by experiments that don't sweep over the field
    /// axis (single-run, grid search, EGO, single/multi-station studies).
    pub fn default_field_config(&self) -> &'static str {
        match self {
            Self::Legacy => "configs/field_configs/legacy/field1.json",
            Self::Vineyard => "configs/field_configs/vineyard/xlarge.json",
        }
    }

    /// Short label used to namespace output directories and filenames.
    pub fn label(&self) -> &'static str {
        match self {
            Self::Legacy => "legacy",
            Self::Vineyard => "vineyard",
        }
    }

    /// Number of completed tasks a single evaluation runs until, before
    /// giving up on reaching a "done" state and reporting whatever it has.
    /// Vineyard's Leo Rover is ~2x slower and has ~6x less battery capacity
    /// than the legacy robot, so 1000 tasks (tuned for the legacy robot)
    /// isn't a realistic target for it -- scaled down accordingly.
    pub fn n_tasks_target(&self) -> u32 {
        match self {
            Self::Legacy => 1000,
            Self::Vineyard => 250,
        }
    }

    /// Parses a `--profile` CLI value. Defaults to `Vineyard` (the prior
    /// behavior) when the flag is absent.
    pub fn from_flag(flag: Option<&str>) -> Self {
        match flag {
            Some("legacy") => Self::Legacy,
            Some("vineyard") | None => Self::Vineyard,
            Some(other) => panic!(
                "unknown --profile value '{other}' (expected 'legacy' or 'vineyard')"
            ),
        }
    }
}
