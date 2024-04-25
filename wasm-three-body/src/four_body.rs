use std::ops;
// use wasm_bindgen::prelude::*;

const STAR_MASS: i32 = 333000;
const EARTH_MASS: i32 = 1;
const G: f64 = 6.67408e-11;

#[derive(PartialEq, Debug)]
struct Acceleration {
    ax: f64,
    ay: f64,
}

impl ops::Add<Acceleration> for Acceleration {
    type Output = Acceleration;
    fn add(self, rhs: Acceleration) -> Acceleration {
        Acceleration {
            ax: self.ax + rhs.ax,
            ay: self.ay + rhs.ay,
        }
    }
}

#[derive(PartialEq, Debug)]
struct AccelerationVelocity {
    ax: f64,
    ay: f64,
    vx: f64,
    vy: f64,
}

impl AccelerationVelocity {
    fn halve(&self) -> AccelerationVelocity {
        AccelerationVelocity {
            ax: self.ax / 2.0,
            ay: self.ay / 2.0,
            vx: self.vx / 2.0,
            vy: self.vy / 2.0,
        }
    }
}

#[derive(PartialEq, Debug)]
struct BodyVelocityPosition {
    mass: f64,
    vx: f64,
    vy: f64,
    x: f64,
    y: f64,
}

impl BodyVelocityPosition {
    // Far-gravity additions should be added to this function
    fn force_from(&self, other: &BodyVelocityPosition) -> Acceleration {
        let dx = other.x - self.x;
        let dy = other.y - self.y;

        let distance = (dx.powi(2) + dy.powi(2)).sqrt();
        // F = g*m*M_other/r**2 = ma
        // a = g*M_other/r**2
        let acceleration_intensity = G * other.mass / distance.powi(2);
        let (sin, cos) = dy.atan2(dx).sin_cos();

        Acceleration {
            ax: acceleration_intensity * cos,
            ay: acceleration_intensity * sin,
        }
    }

    fn cumulative_force_from(
        &self,
        body1: &BodyVelocityPosition,
        body2: &BodyVelocityPosition,
        body3: &BodyVelocityPosition) -> Acceleration {
            self.force_from(body1)
            + self.force_from(body2)
            + self.force_from(body3)
    }

    fn cumulative_force_velocity_from(
        &self,
        body1: &BodyVelocityPosition,
        body2: &BodyVelocityPosition,
        body3: &BodyVelocityPosition) -> AccelerationVelocity {
            let acceleration = self.cumulative_force_from(body1, body2, body3);
            AccelerationVelocity {
                ax: acceleration.ax,
                ay: acceleration.ay,
                vx: self.vx,
                vy: self.vy,
            }
    }
}

impl<'a, 'b> ops::Add<&'b AccelerationVelocity> for &'a BodyVelocityPosition {
    type Output = BodyVelocityPosition;
    fn add(self, rhs: &'b AccelerationVelocity) -> BodyVelocityPosition {
        BodyVelocityPosition {
            mass: self.mass,
            x: self.x + rhs.vx,
            y: self.y + rhs.vy,
            vx: self.vx + rhs.ax,
            vy: self.vy + rhs.ay,
        }
    }
}

#[derive(PartialEq, Debug)]
struct FourAccelerationVelocity {
    star_a: AccelerationVelocity,
    star_b: AccelerationVelocity,
    star_c: AccelerationVelocity,
    planet: AccelerationVelocity,
}

impl FourAccelerationVelocity {
    fn halve(&self) -> FourAccelerationVelocity {
        FourAccelerationVelocity {
            star_a: self.star_a.halve(),
            star_b: self.star_b.halve(),
            star_c: self.star_c.halve(),
            planet: self.planet.halve(),
        }
    }
}

#[derive(PartialEq, Debug)]
struct FourBodyVelocityPosition {
    star_a: BodyVelocityPosition,
    star_b: BodyVelocityPosition,
    star_c: BodyVelocityPosition,
    planet: BodyVelocityPosition,
}

impl FourBodyVelocityPosition {
    fn euler_step_delta(&self) -> FourAccelerationVelocity {
        FourAccelerationVelocity {
            star_a:
                self.star_a.cumulative_force_velocity_from(&self.star_b, &self.star_c, &self.planet),
            star_b:
                self.star_b.cumulative_force_velocity_from(&self.star_a, &self.star_c, &self.planet),
            star_c:
                self.star_c.cumulative_force_velocity_from(&self.star_a, &self.star_b, &self.planet),
            planet:
                self.planet.cumulative_force_velocity_from(&self.star_a, &self.star_b, &self.star_c),
        }
    }

    fn rk4_step_delta(&self) -> FourAccelerationVelocity {
        let k1 = self.euler_step_delta();
        let k2 = (self + &k1.halve()).euler_step_delta();
        let k3 = (self + &k2.halve()).euler_step_delta();
        let k4 = (self + &k3).euler_step_delta();

        k4
    }
}

impl<'a, 'b> ops::Add<&'b FourAccelerationVelocity> for &'a FourBodyVelocityPosition {
    type Output = FourBodyVelocityPosition;
    fn add(self, rhs: &'b FourAccelerationVelocity) -> FourBodyVelocityPosition {
        FourBodyVelocityPosition {
            star_a: &self.star_a + &rhs.star_a,
            star_b: &self.star_b + &rhs.star_b,
            star_c: &self.star_c + &rhs.star_c,
            planet: &self.planet + &rhs.planet,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // #[test]
    // fn add_velocity_position_to_acceleration_velocity() {
    //     let vp = BodyVelocityPosition {
    //         x: 1., y: 2., vx: 3., vy: 4.
    //     };
    //     let av = AccelerationVelocity {
    //         vx: 0.1, vy: 0.2, ax: 0.3, ay: 0.4
    //     };
    //     assert_eq!(vp + av, BodyVelocityPosition {
    //         x: 1.1, y: 2.2, vx: 3.3, vy: 4.4
    //     });
    // }
}
