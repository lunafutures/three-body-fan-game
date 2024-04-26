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

    fn cumulative_force(
        &self,
        body1: &BodyVelocityPosition,
        body2: &BodyVelocityPosition,
        body3: &BodyVelocityPosition) -> Acceleration {
            self.force_from(body1)
            + self.force_from(body2)
            + self.force_from(body3)
    }

    fn cumulative_force_and_velocity(
        &self,
        body1: &BodyVelocityPosition,
        body2: &BodyVelocityPosition,
        body3: &BodyVelocityPosition) -> AccelerationVelocity {
            let acceleration = self.cumulative_force(body1, body2, body3);
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
                self.star_a.cumulative_force_and_velocity(&self.star_b, &self.star_c, &self.planet),
            star_b:
                self.star_b.cumulative_force_and_velocity(&self.star_a, &self.star_c, &self.planet),
            star_c:
                self.star_c.cumulative_force_and_velocity(&self.star_a, &self.star_b, &self.planet),
            planet:
                self.planet.cumulative_force_and_velocity(&self.star_a, &self.star_b, &self.star_c),
        }
    }

    fn rk4_step_delta(&self) -> FourAccelerationVelocity {
        let k1 = self.euler_step_delta();
        let k2 = (self + &k1.halve()).euler_step_delta();
        let k3 = (self + &k2.halve()).euler_step_delta();
        let k4 = (self + &k3).euler_step_delta();

        k4
    }

    pub fn rk4_step(&self) -> FourBodyVelocityPosition {
        self + &self.rk4_step_delta()
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
    use approx::{assert_relative_eq, assert_relative_ne, AbsDiffEq, RelativeEq};

    impl RelativeEq for Acceleration {
        fn default_max_relative() -> f64 {
            f64::default_max_relative()
        }

        fn relative_eq(&self, other: &Self, epsilon: f64, max_relative: f64) -> bool {
            f64::relative_eq(&self.ax, &other.ax, epsilon, max_relative) &&
            f64::relative_eq(&self.ay, &other.ay, epsilon, max_relative)
        }
    }

    impl AbsDiffEq for Acceleration {
        type Epsilon = f64;
        fn default_epsilon() -> Self::Epsilon {
            f64::EPSILON
        }

        fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
            f64::abs_diff_eq(&self.ax, &other.ax, epsilon) &&
            f64::abs_diff_eq(&self.ay, &other.ay, epsilon)
        }
    }

    impl RelativeEq for AccelerationVelocity {
        fn default_max_relative() -> f64 {
            f64::default_max_relative()
        }

        fn relative_eq(&self, other: &Self, epsilon: f64, max_relative: f64) -> bool {
            f64::relative_eq(&self.ax, &other.ax, epsilon, max_relative) &&
            f64::relative_eq(&self.ay, &other.ay, epsilon, max_relative) &&
            f64::relative_eq(&self.vx, &other.vx, epsilon, max_relative) &&
            f64::relative_eq(&self.vy, &other.vy, epsilon, max_relative)
        }
    }

    impl AbsDiffEq for AccelerationVelocity {
        type Epsilon = f64;
        fn default_epsilon() -> Self::Epsilon {
            f64::EPSILON
        }

        fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
            f64::abs_diff_eq(&self.ax, &other.ax, epsilon) &&
            f64::abs_diff_eq(&self.ay, &other.ay, epsilon) &&
            f64::abs_diff_eq(&self.vx, &other.vx, epsilon) &&
            f64::abs_diff_eq(&self.vy, &other.vy, epsilon)
        }
    }

    impl RelativeEq for FourAccelerationVelocity {
        fn default_max_relative() -> f64 {
            f64::default_max_relative()
        }

        fn relative_eq(&self, other: &Self, epsilon: f64, max_relative: f64) -> bool {
            AccelerationVelocity::relative_eq(&self.star_a, &other.star_a, epsilon, max_relative) &&
            AccelerationVelocity::relative_eq(&self.star_b, &other.star_b, epsilon, max_relative) &&
            AccelerationVelocity::relative_eq(&self.star_c, &other.star_c, epsilon, max_relative) &&
            AccelerationVelocity::relative_eq(&self.planet, &other.planet, epsilon, max_relative)
        }
    }

    impl AbsDiffEq for FourAccelerationVelocity {
        type Epsilon = f64;
        fn default_epsilon() -> Self::Epsilon {
            f64::EPSILON
        }

        fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
            AccelerationVelocity::abs_diff_eq(&self.star_a, &other.star_a, epsilon) &&
            AccelerationVelocity::abs_diff_eq(&self.star_b, &other.star_b, epsilon) &&
            AccelerationVelocity::abs_diff_eq(&self.star_c, &other.star_c, epsilon) &&
            AccelerationVelocity::abs_diff_eq(&self.planet, &other.planet, epsilon)
        }
    }

    macro_rules! bvp {
        ($mass:expr, $x:expr, $y:expr) => {
            BodyVelocityPosition {
                mass: $mass,
                x: $x,
                y: $y,
                vx: f64::NAN,
                vy: f64::NAN,
            }
        };
    }

    #[test]
    fn test_acc_add() {
        assert_eq!(
            Acceleration { ax: 1.0, ay: 2.0 } +
            Acceleration { ay: 10.0, ax: 20.0 },
            Acceleration { ax: 21.0, ay: 12.0 }
        );
    }

    #[test]
    fn test_accv_halve() {
        assert_eq!(
            AccelerationVelocity { ax: 0.0, ay: -1.0, vx: 2e10, vy: -4e-12 }.halve(),
            AccelerationVelocity { ax: 0.0, ay: -0.5, vx: 1e10, vy: -2e-12 }
        );
    }

    #[test]
    fn test_bvp_force_from() {
        let body = BodyVelocityPosition { mass: 1., vx: f64::NAN, vy: f64::NAN, x: -1., y: -1. };
        let other = BodyVelocityPosition { mass: 100., vx: f64::NAN, vy: f64::NAN, x: 2., y: 3. };

        assert_relative_eq!(
            body.force_from(&other),
            &Acceleration { ax: 1.6017792000000002e-10, ay: 2.1357056e-10 },
        );
        assert_relative_eq!(
            other.force_from(&body),
            &Acceleration { ax: -1.6017792000000004e-12, ay: -2.1357056e-12 },
        );
    }

    #[test]
    fn test_bvp_cumulative_force_and_velocity() {
        const MASS: f64 = 1.0;
        let body = BodyVelocityPosition { mass: f64::NAN, vx: 1.11, vy: 2.22, x: 0., y: 0.};
        let other1 = bvp!(2. * MASS, 10., 0.);
        let other2 = bvp!(1. * MASS, -10., 0.);
        let other3 = bvp!(1. * MASS, 0., 10.);

        let expected_acceleration = Acceleration { ax: 6.67408e-13, ay: 6.67408e-13 };
        assert_relative_eq!(
            body.cumulative_force(&other1, &other2, &other3),
            &expected_acceleration
        );

        let expected_acceleration_velocity =
            AccelerationVelocity { ax: expected_acceleration.ax, ay: expected_acceleration.ay, vx: 1.11, vy: 2.22 };
        assert_relative_eq!(
            body.cumulative_force_and_velocity(&other1, &other2, &other3),
            &expected_acceleration_velocity
        );
    }

    #[test]
    fn test_bvp_add_acceleration() {
        assert_eq!(
            &BodyVelocityPosition { mass: 1.0, x: 2.0, y: 3.0, vx: 4.0, vy: 5.0 } +
            &AccelerationVelocity { vx: 0.6, vy: 0.7, ax: 0.8, ay: 0.9 },
            BodyVelocityPosition { mass: 1.0, x: 2.6, y: 3.7, vx: 4.8, vy: 5.9 },
        );
    }

    #[test]
    fn test_fav_halve() {
        assert_eq!(
            FourAccelerationVelocity {
                star_a: AccelerationVelocity { ax: 2., ay: 20., vx: 4., vy: 40. },
                star_b: AccelerationVelocity { ax: 4., ay: 20., vx: 4., vy: 40. },
                star_c: AccelerationVelocity { ax: 8., ay: 20., vx: 4., vy: 40. },
                planet: AccelerationVelocity { ax: 16., ay: 20., vx: 4., vy: 40. },
            }.halve(),
            FourAccelerationVelocity {
                star_a: AccelerationVelocity { ax: 1., ay: 10., vx: 2., vy: 20. },
                star_b: AccelerationVelocity { ax: 2., ay: 10., vx: 2., vy: 20. },
                star_c: AccelerationVelocity { ax: 4., ay: 10., vx: 2., vy: 20. },
                planet: AccelerationVelocity { ax: 8., ay: 10., vx: 2., vy: 20. },
            }
        );
    }
}
