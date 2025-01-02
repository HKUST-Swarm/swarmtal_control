all: pc arm64

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build              - build all images"
	# @echo "   1. make pull             - pull all images"
	@echo "   1. make clean              - remove all images"
	@echo ""

arm64:
	@docker build --platform=linux/arm64 -t buaadocker.xuhao1.me/swarmtal_control_ros2:arm64 -f ./Dockerfile .

pc:
	@docker build --pull=false --platform=linux/amd64 -t buaadocker.xuhao1.me/swarmtal_control_ros2:amd64 -f ./Dockerfile .

clean:
	@docker rmi -f swarmtal_control_ros2

upload: upload_pc upload_arm64 

upload_arm64: arm64
	@docker tag swarmtal_control_ros2:arm64 buaadocker.xuhao1.me/swarmtal_control_ros2:arm64
	@docker push buaadocker.xuhao1.me/swarmtal_control_ros2:arm64

upload_pc: pc
	@docker push buaadocker.xuhao1.me/swarmtal_control_ros2:amd64
