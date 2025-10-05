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

## run: run web service
.PHONY: run/web
run/web:
	go run ./cmd/service 

## run: run cli/collect-data
.PHONY: run/cli/collect-data
run/cli/collect-data:
	go run ./cmd/cli/collect-data ./config.json

## run: run cli/sen55-reader
.PHONY: run/cli/sen55-reader
run/cli/sen55-reader:
	go run ./cmd/cli/sen55-reader 00:80:E1:26:06:2E

## run: run cli/mqtt-client
.PHONY: run/cli/mqtt-client
run/cli/mqtt-client:
	go run ./cmd/cli/mqtt-client

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

## build/cli/sen55-reader: build the cmd/cli/sen55-reader application
.PHONY: build/cli/sen55-reader
build/cli/sen55-reader:
	@echo 'Building cmd/cli/sen55-reader...'
	go build -ldflags="-s -w" -gcflags=all=-l -o=./bin/cli/sen55-reader ./cmd/cli/sen55-reader
	GOOS=linux go build -ldflags="-s -w" -gcflags=all=-l -o=./bin/linux/cli/sen55-reader ./cmd/cli/sen55-reader

# ==================================================================================== #
# BUILD DEBUG
# ==================================================================================== #

## build/debug/service: build the cmd/api application
.PHONY: build/debug/service
build/debug/service:
	@echo 'Building cmd/service...'
	go build -gcflags=all="-N -l" -o=./bin/linux/service-debug ./cmd/service

## build/debug/cli/collect-data: build the cmd/cli/collect-data application
.PHONY: build/debug/cli/collect-data
build/debug/cli/collect-data:
	@echo 'Building cmd/cli/collect-data...'
	go build -gcflags=all="-N -l" -o=./bin/cli/collect-data-debug ./cmd/cli/collect-data

## build/debug/cli/sen55-reader: build the cmd/cli/sen55-reader application
.PHONY: build/debug/cli/sen55-reader
build/debug/cli/sen55-reader:
	@echo 'Building cmd/cli/sen55-reader...'
	go build -gcflags=all="-N -l" -o=./bin/cli/sen55-reader-debug ./cmd/cli/sen55-reader

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
