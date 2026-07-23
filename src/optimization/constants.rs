// Define field boundaries for optimization
pub const STATION_MARGIN: f32 = 0.4; // Keep stations at least 0.4m from field edges
pub const OBSTACLE_MARGIN: f32 = 0.4; // Keep stations at least 0.4m from obstacles

// SoC-threshold optimization (Level II) bounds: stay a safety margin above
// the hard critical-SoC floor, cap at a sane upper limit.
pub const SOC_THRESHOLD_MARGIN_ABOVE_CRITICAL: f32 = 5.0;
pub const SOC_THRESHOLD_MAX_PERCENT: f32 = 90.0;
pub const SOC_THRESHOLD_INITIAL_SAMPLES: usize = 6;

// Station-placement EGO budget defaults. Calibrated 2026-07-23 across two
// scenarios (release build; see revisions/AE_2026_R1/TODO.md in the
// agro-charging-framework repo for full numbers):
//  - vineyard profile, n_agents=1: both 1- and 2-station cases converge to
//    within ~0.35% of ground truth at every budget from 30 evaluations up,
//    not a sharp-optimum problem at that scale.
//  - legacy profile, n_agents=4 (the scenario that actually produced the
//    paper's existing Table 9): 2-station converges well (0.7-2.2% of
//    best-found across 30-200 evals) and 30+60 tightens that further; but
//    1-station is genuinely bimodal -- every budget up to 110 evaluations,
//    including 30+60, has a real chance of landing in a local optimum ~14%
//    worse than the grid-search ground truth, and a bigger budget alone
//    does not fix it (needs multi-restart / better initial-DOE coverage,
//    not implemented). Not itself confirmed to be a scenario used in the
//    paper's figures.
// Net choice: 30 initial + 60 BO for the (currently) reliable cases: a
// modest, cheap step up from the legacy-1-station-showed-was-insufficient
// 20+10, without pretending it fixes the 1-station bimodality.
// Override via --ego-initial/--ego-iterations; re-validate if the headline
// fleet size or field set changes.
pub const DEFAULT_EGO_INITIAL_SAMPLES: usize = 30;
pub const DEFAULT_EGO_MAX_ITERATIONS: usize = 60;