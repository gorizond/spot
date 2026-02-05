# LCD Docker Image

Docker image for the LCD service written in C++ with libgpiod GPIO access.

## Building Locally

To build the image locally:

```bash
docker build -t spot-lcd-cpp:latest -f Dockerfile.lcd-cpp .
```

## Running the Container

The container supports different module configurations via command-line arguments:

```bash
# Run all modules
docker run --privileged spot-lcd-cpp:latest -c "/usr/local/bin/spot_lcd_cpp_node -m all"

# Run only temperature and uptime monitors
docker run --privileged spot-lcd-cpp:latest -c "/usr/local/bin/spot_lcd_cpp_node -m temp,uptime"

# Run only LCD and temperature monitor
docker run --privileged spot-lcd-cpp:latest -c "/usr/local/bin/spot_lcd_cpp_node -m lcd,temp"

# Run only the LCD display
docker run --privileged spot-lcd-cpp:latest -c "/usr/local/bin/spot_lcd_cpp_node -m lcd"
```

## GitHub Actions Workflow

This image is automatically built and pushed to GitHub Container Registry when changes are pushed to the main branch. The workflow is defined in `.github/workflows/lcd-image.yml`.

The workflow:
- Builds the Docker image when changes are made to `spot_lcd_cpp/**` or `Dockerfile.lcd-cpp`
- Pushes the image to `ghcr.io/gorizond/spot-lcd-cpp`
- Tags the image with branch name, git SHA, and semantic versions

## Configuration

The LCD service can be configured via the `spot_lcd_cpp/config/lcd_config.toml` file which is copied into the container at build time.

## GPIO Access

The image uses libgpiod for GPIO access instead of wiringPi, providing better compatibility with modern Linux kernels and containerized environments.