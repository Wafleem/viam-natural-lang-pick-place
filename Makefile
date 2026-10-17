# Everyday commands. Python lives in pickplace-module/.venv (make setup).
PY      := pickplace-module/.venv/bin/python
NODE22  := npx -y -p node@22 node
WEB     := code3-web/natural-language-ui
GOAL    ?= box

.PHONY: help setup test test-module test-nl test-dashboard test-web dashboard status plan-check deploy select

help:            ## list targets
	@grep -E '^[a-z-]+:.*## ' $(MAKEFILE_LIST) | sed 's/:.*## /\t/' | sort

setup:           ## create the venv and install everything the tests and tools need
	python3 -m venv pickplace-module/.venv
	$(PY) -m pip install -q -r pickplace-module/requirements.txt -r pickplace-module/requirements-dev.txt -r dashboard/requirements.txt

test: test-module test-nl test-dashboard test-web  ## run every offline test suite (no robot needed)

test-module:     ## pick-and-place module: kinematics, vision, planning, cycle, API
	cd pickplace-module && .venv/bin/python -m pytest -q

test-nl:         ## plain-language parser
	$(PY) -m pytest -q nl-integration

test-dashboard:  ## dashboard server against a fake robot
	$(PY) -m pytest -q dashboard

test-web:        ## code-3 web app (needs Node 22; skipped if not checked out)
	@if [ -f $(WEB)/package.json ]; then cd $(WEB) && npm ci --silent && $(NODE22) build.mjs && $(NODE22) --test test/*.test.mjs; else echo "no $(WEB) on this branch"; fi

dashboard:       ## start the Ultraviolet test bench on http://localhost:8765
	$(PY) dashboard/server.py

status:          ## read-only robot snapshot
	$(PY) pickplace-module/tools/status.py

plan-check:      ## live planner, plan only:  make plan-check GOAL=ring
	$(PY) pickplace-module/tools/plan_check.py --goal $(GOAL)

deploy:          ## upload + cloud build:  make deploy VERSION=0.0.31
	@test -n "$(VERSION)" || (echo "VERSION=0.0.x required" && exit 1)
	$(PY) pickplace-module/tools/deploy.py --version $(VERSION)

select:          ## preview switching code-2:  make select VERSION=0.0.31 [APPLY=1]
	@test -n "$(VERSION)" || (echo "VERSION=0.0.x required" && exit 1)
	$(PY) pickplace-module/tools/select_version.py --version $(VERSION) $(if $(APPLY),--apply,)
