docker buildx build \
  --platform linux/arm64 \
  --pull \
  -f .devcontainer/Dockerfile \
  -t aatmaj9/sookshma-jetson:1.0 \
  --push \
  .

