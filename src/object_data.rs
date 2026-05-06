use std::collections::HashMap;

use serde::Deserialize;

use crate::{SimError, SimResult};

const EMBEDDED_OBJECT_JSON: &str = include_str!("../../gdclone/assets/data/object.json");

#[derive(Debug, Clone, Deserialize)]
pub struct ObjectDefaultData {
    #[serde(default = "default_texture")]
    pub texture: String,
    #[serde(default)]
    pub default_z_layer: i8,
    #[serde(default)]
    pub default_z_order: i16,
    #[serde(default)]
    pub hitbox: Option<RawHitboxData>,
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(tag = "type")]
pub enum RawHitboxData {
    #[serde(rename = "Box")]
    Box {
        #[serde(rename = "x")]
        offset_x: f32,
        #[serde(rename = "y")]
        offset_y: f32,
        width: f32,
        height: f32,
    },
    #[serde(rename = "Slope")]
    Slope { width: f32, height: f32 },
    #[serde(rename = "Circle")]
    Circle { radius: f32 },
}

impl RawHitboxData {
    pub fn normalized(&self) -> HitboxData {
        match *self {
            RawHitboxData::Box {
                offset_x,
                offset_y,
                width,
                height,
            } => HitboxData::Box {
                offset: [offset_x, offset_y],
                half_extents: [width / 2.0, height / 2.0],
            },
            RawHitboxData::Slope { width, height } => HitboxData::Slope {
                half_extents: [width / 2.0, height / 2.0],
            },
            RawHitboxData::Circle { radius } => HitboxData::Circle { radius },
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HitboxData {
    Box {
        offset: [f32; 2],
        half_extents: [f32; 2],
    },
    Slope {
        half_extents: [f32; 2],
    },
    Circle {
        radius: f32,
    },
}

#[derive(Debug, Clone)]
pub struct ObjectDatabase {
    objects: HashMap<u32, ObjectDefaultData>,
}

impl ObjectDatabase {
    pub fn load_embedded() -> SimResult<Self> {
        let objects =
            serde_json::from_str::<HashMap<String, ObjectDefaultData>>(EMBEDDED_OBJECT_JSON)
                .map_err(|error| SimError::ObjectData(error.to_string()))?
                .into_iter()
                .map(|(key, value)| {
                    key.parse::<u32>()
                        .map(|id| (id, value))
                        .map_err(|error| SimError::ObjectData(error.to_string()))
                })
                .collect::<SimResult<HashMap<_, _>>>()?;

        Ok(Self { objects })
    }

    pub fn get(&self, id: u32) -> Option<ObjectDefaultView<'_>> {
        self.objects.get(&id).map(ObjectDefaultView::new)
    }

    pub fn contains(&self, id: u32) -> bool {
        self.objects.contains_key(&id)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ObjectDefaultView<'a> {
    pub texture: &'a str,
    pub default_z_layer: i8,
    pub default_z_order: i16,
    pub hitbox: Option<HitboxData>,
}

impl<'a> ObjectDefaultView<'a> {
    fn new(data: &'a ObjectDefaultData) -> Self {
        Self {
            texture: &data.texture,
            default_z_layer: data.default_z_layer,
            default_z_order: data.default_z_order,
            hitbox: data.hitbox.as_ref().map(RawHitboxData::normalized),
        }
    }
}

fn default_texture() -> String {
    "emptyFrame.png".to_owned()
}
