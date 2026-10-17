"""Reinforcement-learning navigation: environments, policies, learners, evaluation and export.

`docs/plans/rl_policy_navigation_plan.md` holds the design. Each seam (source, policy, learner,
exporter) is a protocol with a registry keyed by name, the way the C++ factories key on `type`.
"""
