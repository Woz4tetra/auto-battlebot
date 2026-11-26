#!/bin/bash
dot -Tsvg docs/diagrams/system.dot -o docs/diagrams/system.svg
dot -Tsvg docs/diagrams/robot_filter.dot -o docs/diagrams/robot_filter.svg
dot -Tsvg docs/diagrams/plant_backed_control.dot -o docs/diagrams/plant_backed_control.svg
dot -Tsvg docs/diagrams/forgejo_ci_topology.dot -o docs/diagrams/forgejo_ci_topology.svg
dot -Tsvg docs/diagrams/forgejo_ci_push_run.dot -o docs/diagrams/forgejo_ci_push_run.svg
dot -Tsvg docs/diagrams/forgejo_ci_report_back.dot -o docs/diagrams/forgejo_ci_report_back.svg
