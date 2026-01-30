# LCD Docker Image

Docker image for the LCD service written in Rust with ROS2 integration.

## Building Locally

To build the image locally:

```bash
docker build -t spot_lcd_rust:latest -f docker/lcd/Dockerfile .
```

## Running the Container

The container supports different module configurations via command-line arguments:

```bash
# Run all modules
docker run --privileged spot_lcd_rust:latest --module all

# Run only temperature and uptime monitors
docker run --privileged spot_lcd_rust:latest --module temp,uptime

# Run only LCD and temperature monitor
docker run --privileged spot_lcd_rust:latest --module lcd,temp

# Run only the LCD display
docker run --privileged spot_lcd_rust:latest --module lcd
```

## GitHub Actions Workflow

This image is automatically built and pushed to GitHub Container Registry when changes are pushed to the main branch. The workflow is defined in `.github/workflows/docker-build.yml`.

The workflow:
- Builds the Docker image when changes are made to `spot_lcd_node/**` or `docker/lcd/Dockerfile`
- Pushes the image to `ghcr.io/gorizond/spot-lcd`
- Tags the image with branch name, git SHA, and semantic versions

## Configuration

The LCD service can be configured via the `config/lcd_config.toml` file which is copied into the container at build time.

## ROS2 Integration

The image supports optional ROS2 integration via the `ros2` feature flag. To build with ROS2 support, use:

```bash
# This is handled automatically in the Dockerfile
cargo build --release --features ros2
```