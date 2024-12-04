all: arm64, pc

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build              - build all images"
	# @echo "   1. make pull             - pull all images"
	@echo "   1. make clean              - remove all images"
	@echo ""

arm64:
	@docker build --platform=linux/arm64 -t swarmtal_control:arm64 -f ./Dockerfile .

pc:
	@docker build  --platform=linux/amd64 -t swarmtal_control:amd64 -f ./Dockerfile .

clean:
	@docker rmi -f swarmtal_control

upload: upload_arm64, upload_amd64

upload_arm64: arm64
	@docker tag swarmtal_control:arm64 buaadocker.xuhao1.me/swarmtal_control:arm64
	@docker push buaadocker.xuhao1.me/swarmtal_control:arm64

upload_amd64: pc
	@docker tag swarmtal_control:amd64 buaadocker.xuhao1.me/swarmtal_control:amd64
	@docker push buaadocker.xuhao1.me/swarmtal_control:amd64
