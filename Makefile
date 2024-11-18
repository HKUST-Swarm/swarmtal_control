all: arm64

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build              - build all images"
	# @echo "   1. make pull             - pull all images"
	@echo "   1. make clean              - remove all images"
	@echo ""

arm64:
	@docker build --platform=linux/arm64 -t buaaswarm/swarmtal_control -f ./Dockerfile .

pc:
	@docker build  --platform=linux/amd64 -t buaaswarm/swarmtal_control -f ./Dockerfile .

clean:
	@docker rmi -f buaaswarm/swarmtal_control
