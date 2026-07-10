from __future__ import annotations

from typing import Any


class MinioObjectStore:
    def __init__(
        self,
        bucket: str,
        *,
        endpoint_url: str,
        access_key: str,
        secret_key: str,
        region: str = "us-east-1",
    ):
        try:
            import boto3  # pyright: ignore[reportMissingImports]
        except ImportError as e:
            raise ImportError(
                "MinioObjectStore requires 'boto3' — "
                "run `uv sync --group infra-minio`"
            ) from e

        self._bucket = bucket
        self._client: Any = boto3.client(
            "s3",
            endpoint_url=endpoint_url,
            aws_access_key_id=access_key,
            aws_secret_access_key=secret_key,
            region_name=region,
        )

    def put(self, key: str, data: bytes) -> None:
        self._client.put_object(Bucket=self._bucket, Key=key, Body=data)

    def get(self, key: str) -> bytes:
        try:
            res = self._client.get_object(Bucket=self._bucket, Key=key)
        except self._client.exceptions.NoSuchKey as e:
            raise KeyError(key) from e
        return res["Body"].read()

    def delete(self, key: str) -> None:
        # S3 delete is idempotent; ObjectStore contract requires KeyError on missing.
        try:
            self._client.head_object(Bucket=self._bucket, Key=key)
        except self._client.exceptions.ClientError as e:
            if e.response.get("Error", {}).get("Code") in ("404", "NoSuchKey"):
                raise KeyError(key) from e
            raise
        self._client.delete_object(Bucket=self._bucket, Key=key)

    def list(self, prefix: str) -> list[str]:
        keys: list[str] = []
        paginator = self._client.get_paginator("list_objects_v2")
        for page in paginator.paginate(Bucket=self._bucket, Prefix=prefix):
            for obj in page.get("Contents", []):
                keys.append(obj["Key"])
        return sorted(keys)
