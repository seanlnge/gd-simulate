use std::env;

use gd_real_sim::{
    level::Level,
    object_data::ObjectDatabase,
    save::{decode_level_payload, read_local_levels, select_local_level},
};

fn main() -> anyhow::Result<()> {
    let mut args = env::args().skip(1);
    let dat = args.next().unwrap();
    let name = args.next().unwrap();
    let x_min: f32 = args.next().unwrap().parse()?;
    let x_max: f32 = args.next().unwrap().parse()?;
    let levels = read_local_levels(std::path::Path::new(&dat))?;
    if name == "--list" {
        for (i, l) in levels.iter().enumerate() {
            println!("[{i}] {:?}", l.name);
        }
        return Ok(());
    }
    let level = select_local_level(&levels, Some(&name))?;
    let levelstring = if level.levelstring.starts_with('H') || level.levelstring.starts_with('C') {
        decode_level_payload(&level.raw_payload)?
    } else {
        level.levelstring.clone()
    };
    let db = ObjectDatabase::load_embedded()?;
    let parsed = Level::parse(&levelstring, &db)?;
    let mut hits: Vec<_> = parsed
        .objects
        .iter()
        .filter(|o| o.x >= x_min && o.x <= x_max)
        .collect();
    hits.sort_by(|a, b| a.x.partial_cmp(&b.x).unwrap());
    println!("objects in x ∈ [{x_min}, {x_max}] for {name}:");
    for o in hits {
        let texture = db
            .get(o.object_id)
            .map(|view| view.texture)
            .unwrap_or("unknown");
        let hb = o
            .hitbox
            .map(|h| format!("{:?}", h))
            .unwrap_or_else(|| "no-hitbox".to_owned());
        println!(
            "  id={} name={} x={:.1} y={:.1} rot={:.0}° kind={:?} hitbox={} raw(4/5/32/128/129)=({:?}/{:?}/{:?}/{:?}/{:?})",
            o.object_id,
            texture,
            o.x,
            o.y,
            o.rotation,
            o.kind,
            hb,
            o.raw.get("4"),
            o.raw.get("5"),
            o.raw.get("32"),
            o.raw.get("128"),
            o.raw.get("129"),
        );
    }
    Ok(())
}
