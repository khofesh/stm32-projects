# ==================================================================================== #
# HELPERS
# ==================================================================================== #

## help: print this help message
.PHONY: help
help:
	@echo 'Usage:'
	@sed -n 's/^##//p' ${MAKEFILE_LIST} | column -t -s ':' | sed -e 's/^/ /'

# ==================================================================================== #
# DEVELOPMENT
# ==================================================================================== #

## run: run service
.PHONY: run/service
run/service:
	go run ./cmd/service 

## run: run cli
.PHONY: run/cli
run/cli:
	go run ./cmd/cli

# ==================================================================================== #
# BUILD
# ==================================================================================== #

## build/service: build the cmd/api application
.PHONY: build/service
build/service:
	@echo 'Building cmd/service...'
	go build -ldflags="-s -w" -gcflags=all=-l -o=./bin/service ./cmd/service
	GOOS=linux go build -ldflags="-s -w" -gcflags=all=-l -o=./bin/linux/service ./cmd/service

## build/cli/collect-data: build the cmd/cli/collect-data application
.PHONY: build/cli/collect-data
build/cli/collect-data:
	@echo 'Building cmd/cli/collect-data...'
	go build -ldflags="-s -w" -gcflags=all=-l -o=./bin/cli/collect-data ./cmd/cli/collect-data
	GOOS=linux go build -ldflags="-s -w" -gcflags=all=-l -o=./bin/linux/cli/collect-data ./cmd/cli/collect-data

# ==================================================================================== #
# BUILD DEBUG
# ==================================================================================== #

## build/debug/service: build the cmd/api application
.PHONY: build/debug/service
build/debug/service:
	@echo 'Building cmd/service...'
	go build -gcflags=all="-N -l" -o=./bin/linux/service ./cmd/service

## build/debug/cli/collect-data: build the cmd/cli/collect-data application
.PHONY: build/debug/cli/collect-data
build/debug/cli/collect-data:
	@echo 'Building cmd/cli/collect-data...'
	go build -gcflags=all="-N -l" -o=./bin/cli/collect-data ./cmd/cli/collect-data

# ==================================================================================== #
# QUALITY CONTROL
# ==================================================================================== #

## audit: tidy dependencies and format, vet and test all code
.PHONY: audit
audit: tidy
	@echo 'Formatting code...'
	go fmt ./...
	@echo 'Vetting code...'
	go vet ./...
	staticcheck ./...
	@echo 'Running tests...'
	go test -race -vet=off ./...

## vendor: tidy and vendor dependencies
.PHONY: vendor 
vendor:
	@echo 'Tidying and verifying module dependencies...'
	go mod tidy 
	go mod verify 
	@echo 'Vendoring dependencies...'
	go mod vendor

.PHONY: tidy 
tidy:
	@echo 'Tidying and verifying module dependencies...'
	go mod tidy 
	go mod verify 
