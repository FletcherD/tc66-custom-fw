//! Proves the firmware's fixed-point NTC conversion (rust/src/ntc.rs,
//! compiled here verbatim) bit-identical to the stock f32 model over every
//! possible ADC sum16, and reports both against the exact f64 model.

#[path = "../../../rust/src/ntc.rs"]
mod ntc;

/// The stock model in f32 + libm logf, as the firmware computed it before
/// the fixed-point port.
fn stock_f32(sum16: u32) -> i32 {
    let avg = sum16 as f32 / 16.0;
    if avg < 1.0 || avg > 4094.0 {
        return 0;
    }
    let r = avg * 11500.0 / (4095.0 - avg);
    let inv_t = 1.0 / 298.15 + libm::logf(r / 10_000.0) / 3450.0;
    (1.0 / inv_t - 273.15) as i32
}

fn exact(sum16: u32) -> f64 {
    let avg = sum16 as f64 / 16.0;
    if !(1.0..=4094.0).contains(&avg) {
        return 0.0;
    }
    let r = avg * 11500.0 / (4095.0 - avg);
    1.0 / (1.0 / 298.15 + (r / 10_000.0).ln() / 3450.0) - 273.15
}

fn main() {
    let (mut vs_stock, mut vs_exact) = (0u32, 0u32);
    for s in 0..=65535u32 {
        let v = ntc::temp_c(s);
        if v != stock_f32(s) {
            vs_stock += 1;
            println!("  sum16={s}: fixed={v} stock_f32={} exact={:.6}", stock_f32(s), exact(s));
        }
        if v as f64 != exact(s).trunc() {
            vs_exact += 1;
        }
    }
    println!("sum16 0..=65535 ({}..={} C): {vs_stock} differ from stock f32, {vs_exact} from trunc(exact)",
             ntc::temp_c(65504), ntc::temp_c(16));
    std::process::exit((vs_stock != 0) as i32);
}
