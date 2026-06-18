//! `rosbridge_websocket` — pure-Rust rosbridge_server replacement.

use std::sync::Arc;

use clap::Parser;
use ros_message::Registry;
use rosbridge_server::backend::loopback::LoopbackBackend;
use rosbridge_server::backend::SharedBackend;
use rosbridge_server::config::{parse_glob_string, Config};
use rosbridge_server::server::Server;

/// Command-line arguments mirroring the rosbridge_websocket node parameters.
#[derive(Parser, Debug)]
#[command(name = "rosbridge_websocket", version, about = "Pure-Rust rosbridge_server")]
struct Args {
    #[arg(long, default_value_t = 9090)]
    port: u16,
    #[arg(long, default_value = "")]
    address: String,
    #[arg(long, default_value = "/")]
    url_path: String,
    #[arg(long, default_value_t = 2.0)]
    retry_startup_delay: f64,
    #[arg(long, default_value = "")]
    certfile: String,
    #[arg(long, default_value = "")]
    keyfile: String,
    #[arg(long, default_value_t = 0.0)]
    websocket_ping_interval: f64,
    #[arg(long, default_value_t = 30.0)]
    websocket_ping_timeout: f64,
    #[arg(long, default_value_t = false)]
    use_compression: bool,

    #[arg(long, default_value_t = 600)]
    fragment_timeout: u64,
    #[arg(long, default_value_t = 0.0)]
    delay_between_messages: f64,
    #[arg(long, default_value_t = 1_000_000)]
    max_message_size: usize,
    #[arg(long, default_value_t = 10.0)]
    unregister_timeout: f64,
    #[arg(long, default_value = "")]
    topics_glob: String,
    #[arg(long, default_value = "")]
    topics_pub_glob: String,
    #[arg(long, default_value = "")]
    topics_sub_glob: String,
    #[arg(long, default_value = "")]
    services_glob: String,
    #[arg(long, default_value = "")]
    actions_glob: String,
    #[arg(long, default_value_t = true)]
    call_services_in_new_thread: bool,
    #[arg(long, default_value_t = 5.0)]
    default_call_service_timeout: f64,
    #[arg(long, default_value_t = true)]
    send_action_goals_in_new_thread: bool,

    /// Extra ament-prefix paths (colon-separated) to scan for interface defs.
    /// Defaults to `$AMENT_PREFIX_PATH` when present.
    #[arg(long)]
    interface_paths: Option<String>,

    /// ROS backend to use. `dds` requires the `dds` build feature.
    #[arg(long, default_value = "loopback")]
    backend: String,
}

fn build_config(args: &Args) -> Config {
    let mut cfg = Config {
        port: args.port,
        address: args.address.clone(),
        url_path: args.url_path.clone(),
        retry_startup_delay: args.retry_startup_delay,
        certfile: args.certfile.clone(),
        keyfile: args.keyfile.clone(),
        websocket_ping_interval: args.websocket_ping_interval,
        websocket_ping_timeout: args.websocket_ping_timeout,
        use_compression: args.use_compression,
        fragment_timeout: args.fragment_timeout,
        delay_between_messages: args.delay_between_messages,
        max_message_size: args.max_message_size,
        unregister_timeout: args.unregister_timeout,
        topics_glob: parse_glob_string(&args.topics_glob),
        topics_pub_glob: parse_glob_string(&args.topics_pub_glob),
        topics_sub_glob: parse_glob_string(&args.topics_sub_glob),
        services_glob: parse_glob_string(&args.services_glob),
        actions_glob: parse_glob_string(&args.actions_glob),
        call_services_in_new_thread: args.call_services_in_new_thread,
        default_call_service_timeout: args.default_call_service_timeout,
        send_action_goals_in_new_thread: args.send_action_goals_in_new_thread,
        interface_paths: Vec::new(),
    };
    cfg.finalize_globs();
    cfg
}

fn build_registry(args: &Args) -> Registry {
    let mut reg = Registry::with_standard_types();
    let paths = args
        .interface_paths
        .clone()
        .or_else(|| std::env::var("AMENT_PREFIX_PATH").ok())
        .unwrap_or_default();
    for p in paths.split(':').filter(|s| !s.is_empty()) {
        match reg.load_ament_prefix(std::path::Path::new(p)) {
            Ok(n) if n > 0 => tracing::info!("loaded {n} interface definitions from {p}"),
            Ok(_) => {}
            Err(e) => tracing::warn!("failed to scan {p}: {e}"),
        }
    }
    tracing::info!("registry holds {} message types", reg.message_count());
    reg
}

fn build_backend(name: &str, registry: &Arc<Registry>) -> anyhow::Result<SharedBackend> {
    let _ = registry; // used only by the rcl backend
    match name {
        "loopback" => Ok(Arc::new(LoopbackBackend::new())),
        #[cfg(feature = "rcl")]
        "rcl" => Ok(rosbridge_server::backend::rcl::RclBackend::shared(registry.clone())?),
        other => anyhow::bail!(
            "unknown backend '{other}' (available: loopback{})",
            if cfg!(feature = "rcl") { ", rcl" } else { "" }
        ),
    }
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "info".into()),
        )
        .init();

    let args = Args::parse();
    if args.certfile.is_empty() != args.keyfile.is_empty() {
        tracing::warn!("both --certfile and --keyfile are required to enable SSL; ignoring");
    }
    let cfg = Arc::new(build_config(&args));
    let registry = Arc::new(build_registry(&args));
    let backend = build_backend(&args.backend, &registry)?;

    let server = Server::new(cfg, registry, backend);
    server.serve(|addr| tracing::info!("bound to {addr}")).await?;
    Ok(())
}
