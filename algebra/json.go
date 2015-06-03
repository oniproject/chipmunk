package vect 

import (
	"encoding/json"
	"log"
)

func (v Vect) MarshalJSON() ([]byte, error) {
	return json.Marshal(&[2]Float{v.X, v.Y})
}

func (v *Vect) UnmarshalJSON(data []byte) error {
	vectData := [2]Float{}

	//try unmarshalling array form
	err := json.Unmarshal(data, &vectData)
	if err != nil {
		//try other form
		vectData := struct {
			X, Y Float
		}{}

		err := json.Unmarshal(data, &vectData)

		if err != nil {
			log.Printf("Error decoding Vect")
			return err
		}
		v.X = vectData.X
		v.Y = vectData.Y
		return nil
	}

	v.X = vectData[0]
	v.Y = vectData[1]

	return nil
}

func (xf Transform) MarshalJSON() ([]byte, error) {
	xfData := struct {
		Position Vect
		Rotation Float
	}{
		Position: xf.Position,
		Rotation: xf.Angle(),
	}

	return json.Marshal(&xfData)
}

func (xf *Transform) UnmarshalJSON(data []byte) error {
	xf.SetIdentity()

	xfData := struct {
		Position *Vect
		Rotation Float
	}{
		Position: &xf.Position,
		Rotation: xf.Angle(),
	}

	err := json.Unmarshal(data, &xfData)
	if err != nil {
		log.Printf("Error decoding transform")
		return err
	}

	xf.Position = *xfData.Position
	xf.SetAngle(xfData.Rotation)

	return nil
}