# builds with --no-cache to update the git informations, in case new infos were pushed (otherwise takes from docker cache)

docker build --no-cache --build-arg GIT_USER=Yanniscod --build-arg GIT_TOKEN=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 -t username/img_name:tag Desktop/SwissCat-on_robot