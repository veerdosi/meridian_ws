docker build -t meridian:latest /Users/veerdosi/Documents/code/github/meridian_ws

docker run -it --name meridian_dev --platform linux/arm64 -v /Users/veerdosi/Documents/code/github/meridian_ws:/root/meridian_ws -p 5900:5900 -e LIBGL_ALWAYS_SOFTWARE=1 meridian:latest
