docker buildx build \
  --platform linux/arm64 \
  --pull \
  --no-cache \
  -f .devcontainer/Dockerfile \
  -t aatmaj9/sookshma-jetson:2.0 \
  --push \
  .