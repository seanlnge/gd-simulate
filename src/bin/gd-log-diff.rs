use std::{fs, path::PathBuf};

use clap::Parser;
use gd_real_sim::trace_diff::{DiffConfig, compare_logs};

#[derive(Debug, Parser)]
#[command(name = "gd-log-diff")]
#[command(about = "Compare real Geometry Dash tick logs against gd-real-sim tick logs")]
struct Args {
    #[arg(long)]
    real_log: PathBuf,
    #[arg(long)]
    sim_log: PathBuf,
    #[arg(long, default_value_t = DiffConfig::default().epsilon_x)]
    epsilon_x: f32,
    #[arg(long, default_value_t = DiffConfig::default().epsilon_y)]
    epsilon_y: f32,
    #[arg(long, default_value_t = DiffConfig::default().min_motion)]
    min_motion: f32,
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();
    let real_log = fs::read_to_string(args.real_log)?;
    let sim_log = fs::read_to_string(args.sim_log)?;
    let report = compare_logs(
        &real_log,
        &sim_log,
        DiffConfig {
            epsilon_x: args.epsilon_x,
            epsilon_y: args.epsilon_y,
            min_motion: args.min_motion,
        },
    )
    .map_err(anyhow::Error::msg)?;

    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(())
}
