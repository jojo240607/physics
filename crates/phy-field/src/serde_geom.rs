//! 自定义 serde 模块:把 nalgebra 泛型 `Vec3<T>` / `Vec<Vec3<T>>` 序列化为纯元组,
//! 绕过 nalgebra 自带的 `Matrix<T>: Serialize`(要求 `T: nalgebra::Scalar`)带来的
//! impl 传播问题。本模块仅依赖 `phy_math`,不引入循环依赖。

use phy_math::Vec3;
use serde::{Deserialize, Deserializer, Serialize, Serializer};

pub fn serialize<S: Serializer, T: Serialize + Copy>(v: &Vec3<T>, s: S) -> Result<S::Ok, S::Error> {
    [v[0], v[1], v[2]].serialize(s)
}

pub fn deserialize<'de, D: Deserializer<'de>, T: Deserialize<'de> + Copy>(
    d: D,
) -> Result<Vec3<T>, D::Error> {
    let a = <[T; 3]>::deserialize(d)?;
    Ok(Vec3::new(a[0], a[1], a[2]))
}

pub mod vec3_vec {
    use phy_math::Vec3;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    pub fn serialize<S: Serializer, T: Serialize + Copy>(
        v: &Vec<Vec3<T>>,
        s: S,
    ) -> Result<S::Ok, S::Error> {
        v.iter().map(|x| [x[0], x[1], x[2]]).collect::<Vec<_>>().serialize(s)
    }

    pub fn deserialize<'de, D: Deserializer<'de>, T: Deserialize<'de> + Copy>(
        d: D,
    ) -> Result<Vec<Vec3<T>>, D::Error> {
        let arr = <Vec<[T; 3]>>::deserialize(d)?;
        Ok(arr.into_iter().map(|a| Vec3::new(a[0], a[1], a[2])).collect())
    }
}
