use std::env;

use gd_real_sim::{
    level::Level,
    object_data::ObjectDatabase,
    save::{decode_level_payload, read_local_levels, select_local_level},
};

fn main() -> anyhow::Result<()> {
    let mut args = env::args().skip(1);
    let dat = args
        .next()
        .expect("usage: dump_header <CCLocalLevels.dat> <name>");
    let name = args
        .next()
        .expect("usage: dump_header <CCLocalLevels.dat> <name>");
    let levels = read_local_levels(std::path::Path::new(&dat))?;
    let level = select_local_level(&levels, Some(&name))?;
    let levelstring = if level.levelstring.starts_with('H') || level.levelstring.starts_with('C') {
        decode_level_payload(&level.raw_payload)?
    } else {
        level.levelstring.clone()
    };
    let db = ObjectDatabase::load_embedded()?;
    let parsed = Level::parse(&levelstring, &db)?;
    println!("name: {}", level.name);
    println!("header keys: {}", parsed.header.len());
    let mut keys: Vec<_> = parsed.header.iter().collect();
    keys.sort_by_key(|(k, _)| *k);
    for (k, v) in keys {
        println!("  {k} = {v}");
    }
    println!("objects: {}", parsed.objects.len());
    let mut object_id_counts = std::collections::BTreeMap::<u32, u32>::new();
    for obj in &parsed.objects {
        *object_id_counts.entry(obj.object_id).or_insert(0) += 1;
    }
    let mut top: Vec<_> = object_id_counts.iter().collect();
    top.sort_by(|a, b| b.1.cmp(a.1));
    println!("top object ids:");
    for (id, count) in top.iter().take(15) {
        println!("  id {id}: {count}");
    }
    Ok(())
}
