"""Linearize the point-mass cart-pole equations at the upright state."""

from __future__ import annotations

import sympy as sp


def main() -> None:
    x, theta, x_dot, theta_dot, force = sp.symbols("x theta x_dot theta_dot force")
    cart_mass, pole_mass, length, gravity = sp.symbols(
        "cart_mass pole_mass length gravity", positive=True
    )

    mass_matrix = sp.Matrix(
        [
            [cart_mass + pole_mass, -pole_mass * length * sp.cos(theta)],
            [-pole_mass * length * sp.cos(theta), pole_mass * length**2],
        ]
    )
    # MuJoCo's +Y hinge convention gives x_pole = x - l*sin(theta).
    coriolis = sp.Matrix([pole_mass * length * theta_dot**2 * sp.sin(theta), 0])
    gravity_vector = sp.Matrix([0, pole_mass * gravity * length * sp.sin(theta)])
    generalized_force = sp.Matrix([force, 0])
    acceleration = mass_matrix.inv() * (generalized_force - coriolis - gravity_vector)

    state = sp.Matrix([x, theta, x_dot, theta_dot])
    vector_field = sp.Matrix([x_dot, theta_dot, acceleration[0], acceleration[1]])
    a_symbolic = vector_field.jacobian(state)
    b_symbolic = vector_field.jacobian([force])

    upright = {
        x: 0,
        theta: sp.pi,
        x_dot: 0,
        theta_dot: 0,
        force: 0,
        cart_mass: 1.0,
        pole_mass: 1.0,
        length: 0.5,
        gravity: 9.81,
    }
    print("Continuous-time A_c at upright:")
    sp.pprint(sp.N(a_symbolic.subs(upright), 6))
    print("\nContinuous-time B_c at upright:")
    sp.pprint(sp.N(b_symbolic.subs(upright), 6))


if __name__ == "__main__":
    main()
