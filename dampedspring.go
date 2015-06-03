package chipmunk

import (
	. "github.com/oniproject/chipmunk/algebra"

	"math"
)

type DampedSpring struct {
	BasicConstraint

	Anchor1, Anchor2 Vect
	RestLength       Float
	Stiffness        Float
	Damping          Float
	SpringForceFunc  func(*DampedSpring, Float) Float

	targetVRN Float
	vCoef     Float

	r1, r2 Vect
	nMass  Float
	n      Vect
}

func defaultSpringForce(spring *DampedSpring, dist Float) Float {
	return (spring.RestLength - dist) * spring.Stiffness
}

func NewDampedSpring(a, b *Body,
	anchor1, anchor2 Vect,
	restLength, stiffness, damping Float) *DampedSpring {
	return &DampedSpring{
		BasicConstraint: NewConstraint(a, b),
		Anchor1:         anchor1,
		Anchor2:         anchor2,
		SpringForceFunc: defaultSpringForce,
		RestLength:      restLength,
		Stiffness:       stiffness,
		Damping:         damping,
	}
}

func (spring *DampedSpring) PreStep(dt Float) {
	a := spring.BodyA
	b := spring.BodyB

	spring.r1 = RotateVect(spring.Anchor1, Rotation{a.rot.X, a.rot.Y})
	spring.r2 = RotateVect(spring.Anchor2, Rotation{a.rot.X, a.rot.Y})

	delta := Sub(Add(b.p, spring.r2), Add(a.p, spring.r1))
	dist := Length(delta)
	if dist == 0 {
		dist = Float(math.Inf(1))
	}
	spring.n = Mult(delta, 1.0/dist)

	k := k_scalar(a, b, spring.r1, spring.r2, spring.n)
	spring.nMass = 1.0 / k

	spring.targetVRN = 0.0
	spring.vCoef = Float(1.0 - math.Exp(float64(-spring.Damping*dt*k)))

	fSpring := spring.SpringForceFunc(spring, dist)
	apply_impulses(a, b, spring.r1, spring.r2, Mult(spring.n, fSpring*dt))
}

func (spring *DampedSpring) ApplyCachedImpulse(_ Float) {}

func (spring *DampedSpring) ApplyImpulse() {
	a := spring.BodyA
	b := spring.BodyB

	n := spring.n
	r1 := spring.r1
	r2 := spring.r2

	vrn := normal_relative_velocity(a, b, r1, r2, n)

	vDamp := (spring.targetVRN - vrn) * spring.vCoef
	spring.targetVRN = vrn + vDamp

	apply_impulses(a, b, spring.r1, spring.r2, Mult(spring.n, vDamp*spring.nMass))
}

func (spring *DampedSpring) Impulse() Float {
	return 0
}
