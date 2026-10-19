"""Only test/unit holds pytest tests.

test/probes need a running simulation and test/validation are offline scripts
with their own command-line options; run both by hand with python3.
"""
collect_ignore = ["probes", "validation"]
