import csv
import json

from wpimath import geometry, units
import numpy as np


filename = "tag_pos.csv"

json_data = {"fiducials" : []}

csv_file = open(filename, "r")
reader = csv.reader(csv_file, delimiter=",")

for row in reader:
	id = int(row[0])
	x = float(row[1])
	y = float(row[2])
	z = float(row[3])
	rotation = float(row[4])
	
	q = geometry.Rotation3d(0,0,units.degreesToRadians(rotation)).getQuaternion()
	
	json_data["fiducials"].append(
				{
		"family": "apriltag3_36h11_classic",
		"id": id,
		"size": 160.0,
		"unique" : 1,
		"transform" : [
			q.W(),
			0,
			0,
			units.inchesToMeters(x),
			0,
			q.X(),
			0,
			units.inchesToMeters(y),
			0,
			0,
			q.Y(),
			units.inchesToMeters(z),
			0,
			0,
			0,
			q.Z()
			],
		},
	)

f = open("result_json.fmap","w")

json.dump(json_data, f, indent=2)