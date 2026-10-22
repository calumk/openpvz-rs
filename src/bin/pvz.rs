//! Command-line entry point. Keep parsing/format logic out of parsers.
use clap::{ArgAction, Parser, Subcommand};
use openpvz::{
    diagnostics, export_glb, export_step, export_step_ol, tools, version::VERSION,
    PvzArchive,
    constants::{DEFAULT_MIN_SLIVER_WIDTH, DEFAULT_PCURVE_TOLERANCE, DEFAULT_STEP_UNCERTAINTY},
};

#[derive(Parser)]
#[command(name = "pvz", version = VERSION, about = "Standalone PTC Creo View PVZ/PVS/OL converter")]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    Info { file: String },
    Tree { file: String, #[arg(long)] depth: Option<usize> },
    Validate { file: String, #[arg(long)] json: Option<String> },
    Convert {
        file: String,
        output: String,
        #[arg(long, default_value_t = DEFAULT_PCURVE_TOLERANCE)]
        pcurve_tolerance: f64,
        /// Alias for --pcurve-tolerance.
        #[arg(long)]
        tolerance: Option<f64>,
        #[arg(long, default_value_t = DEFAULT_STEP_UNCERTAINTY)]
        step_uncertainty: f64,
        #[arg(long, default_value_t = DEFAULT_MIN_SLIVER_WIDTH)]
        min_sliver_width: f64,
        #[arg(long, action = ArgAction::SetTrue)]
        include_construction: bool,
    },
    Obj { file: String, member: String, output: String },
    #[command(name = "step-ol")]
    StepOl {
        file: String,
        member: String,
        output: String,
        #[arg(long, default_value_t = DEFAULT_PCURVE_TOLERANCE)]
        pcurve_tolerance: f64,
        /// Alias for --pcurve-tolerance.
        #[arg(long)]
        tolerance: Option<f64>,
        #[arg(long, default_value_t = DEFAULT_STEP_UNCERTAINTY)]
        step_uncertainty: f64,
        #[arg(long, default_value_t = DEFAULT_MIN_SLIVER_WIDTH)]
        min_sliver_width: f64,
    },
    #[command(name = "probe-ol")]
    ProbeOl { file: String },
    Inspect { file: String, #[arg(long, default_value_t = 5)] sample_ol: usize },
    Dump { file: String, dir: String, #[arg(long)] ol: Option<String> },
    Exact { file: String, member: String, #[arg(long, action = ArgAction::SetTrue)] validate: bool, #[arg(long, default_value_t = DEFAULT_PCURVE_TOLERANCE)] tolerance: f64 },
    #[command(name = "probe-exact")]
    ProbeExact { file: String },
    #[command(name = "validate-exact")]
    ValidateExact { file: String, #[arg(long, default_value_t = DEFAULT_PCURVE_TOLERANCE)] tolerance: f64, #[arg(long)] json: Option<String> },
    #[command(name = "occ-json")]
    OccJson { file: String, member: String, output: String, #[arg(long, action = ArgAction::SetTrue)] include_normalized_pcurves: bool },
}

fn progress(label: &str) -> impl Fn(usize, usize) + '_ {
    let last = std::cell::Cell::new(-1i64);
    move |done: usize, total: usize| {
        let pct = (done * 100 / total.max(1)) as i64;
        if pct >= last.get() + 10 || done == total {
            eprintln!("{label}: {done}/{total} ({pct}%)");
            last.set(pct);
        }
    }
}

fn print_json(v: &serde_json::Value) {
    println!("{}", serde_json::to_string_pretty(v).unwrap());
}

fn main() {
    if let Err(e) = run() {
        eprintln!("error: {e}");
        if std::env::var("PVZ_DEBUG").is_ok() {
            eprintln!("{e:?}");
        }
        std::process::exit(1);
    }
}

fn run() -> openpvz::Result<()> {
    let cli = Cli::parse();
    match cli.command {
        Commands::Info { file } => {
            let archive = PvzArchive::open(&file)?;
            let stats = diagnostics::model_stats(&archive)?;
            print_json(&serde_json::to_value(stats).unwrap());
        }
        Commands::Tree { file, depth } => {
            let archive = PvzArchive::open(&file)?;
            diagnostics::print_tree(&archive, depth);
        }
        Commands::Validate { file, json } => {
            let archive = PvzArchive::open(&file)?;
            let cb = progress("validating OL");
            let result = diagnostics::validate_archive(&archive, DEFAULT_PCURVE_TOLERANCE, Some(&cb));
            let text = serde_json::to_string_pretty(&result).unwrap();
            if let Some(path) = json {
                std::fs::write(&path, format!("{text}\n")).map_err(|e| openpvz::openpvz_error(format!("cannot write {path}: {e}")))?;
            }
            println!("{text}");
            if !result["ok"].as_bool().unwrap_or(false) {
                std::process::exit(2);
            }
        }
        Commands::Convert { file, output, pcurve_tolerance, tolerance, step_uncertainty, min_sliver_width, include_construction } => {
            let archive = PvzArchive::open(&file)?;
            let lower = output.to_lowercase();
            let tol = tolerance.unwrap_or(pcurve_tolerance);
            if lower.ends_with(".step") || lower.ends_with(".stp") {
                let cb = progress("writing STEP");
                let totals = export_step(&archive, &output, tol, step_uncertainty, min_sliver_width, Some(&cb))?;
                print_json(&serde_json::to_value(totals).unwrap());
            } else if lower.ends_with(".glb") {
                let cb = progress("decoding OL");
                let stats = export_glb(&archive, &output, include_construction, Some(&cb))?;
                print_json(&serde_json::to_value(stats).unwrap());
            } else {
                return Err(openpvz::openpvz_error("output extension must be .step, .stp or .glb"));
            }
        }
        Commands::Obj { file, member, output } => {
            print_json(&tools::export_ol_obj(&file, &member, &output)?);
        }
        Commands::StepOl { file, member, output, pcurve_tolerance, tolerance, step_uncertainty, min_sliver_width } => {
            let archive = PvzArchive::open(&file)?;
            let tol = tolerance.unwrap_or(pcurve_tolerance);
            let (stats, mapping) = export_step_ol(&archive, &member, &output, tol, step_uncertainty, min_sliver_width)?;
            print_json(&serde_json::json!({
                "output": output,
                "faceCount": stats.face_count, "edgeCount": stats.edge_count,
                "vertexCount": stats.vertex_count, "pcurveCount": stats.pcurve_count,
                "closedShells": stats.closed_shells, "openShells": stats.open_shells,
                "appearanceMapping": mapping,
            }));
        }
        Commands::ProbeOl { file } => {
            let cb = progress("probing OL");
            print_json(&tools::probe_ol_archive(&file, Some(&cb))?);
        }
        Commands::Inspect { file, sample_ol } => {
            print_json(&tools::inspect_containers(&file, sample_ol)?);
        }
        Commands::Dump { file, dir, ol } => {
            print_json(&tools::dump_streams(&file, &dir, ol.as_deref())?);
        }
        Commands::Exact { file, member, validate, tolerance } => {
            print_json(&tools::exact_ol_info(&file, &member, validate, tolerance)?);
        }
        Commands::ProbeExact { file } => {
            let cb = progress("probing exact OL");
            print_json(&tools::probe_exact_archive(&file, Some(&cb))?);
        }
        Commands::ValidateExact { file, tolerance, json } => {
            let cb = progress("validating exact OL");
            let result = tools::validate_exact_archive(&file, tolerance, Some(&cb))?;
            let text = serde_json::to_string_pretty(&result).unwrap();
            if let Some(path) = json {
                std::fs::write(&path, format!("{text}\n")).map_err(|e| openpvz::openpvz_error(format!("cannot write {path}: {e}")))?;
            }
            println!("{text}");
            if !result["ok"].as_bool().unwrap_or(false) {
                std::process::exit(2);
            }
        }
        Commands::OccJson { file, member, output, include_normalized_pcurves } => {
            print_json(&tools::export_occ_intermediate(&file, &member, &output, include_normalized_pcurves)?);
        }
    }
    Ok(())
}
