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
	@docker build  -t buaaswarm/swarmtal_control -f ./Dockerfile .

clean:
	@docker rmi -f buaaswarm/swarmtal_control
