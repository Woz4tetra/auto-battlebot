#!/bin/bash
dot -Tsvg docs/diagrams/system.dot -o docs/diagrams/system.svg
dot -Tsvg docs/diagrams/robot_filter.dot -o docs/diagrams/robot_filter.svg
dot -Tsvg docs/diagrams/plant_backed_control.dot -o docs/diagrams/plant_backed_control.svg
