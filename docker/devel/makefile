build:
	docker build -f Dockerfile.devel . -t swarmslam_viz_devel

start:
	docker start swarmslam_viz_devel

stop:
	docker stop swarmslam_viz_devel

run:
	docker run -itd --net host -v $(PWD)/../../:/cslam_visualization --env="DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix --name swarmslam_viz_devel swarmslam_viz_devel
#	docker run -itd --network=none -v $(PWD)/../../../:/cslam_visualization --env="DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix --name swarmslam_viz_devel swarmslam_viz_devel

kill:
	docker kill swarmslam_viz_devel

rm:
	docker rm swarmslam_viz_devel

attach:
	docker exec -it swarmslam_viz_devel bash
