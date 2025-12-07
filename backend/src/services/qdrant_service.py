import os
from typing import List, Dict, Any
from qdrant_client import QdrantClient, models
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

class QdrantService:
    def __init__(self, collection_name: str):
        self.collection_name = collection_name
        self.qdrant_url = os.getenv("QDRANT_URL")
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if not self.qdrant_url or not self.qdrant_api_key:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables must be set.")

        self.client = QdrantClient(
            url=self.qdrant_url,
            api_key=self.qdrant_api_key,
        )
        self.vector_size = 1536  # OpenAI text-embedding-ada-002 typically produces 1536-dimensional vectors

    def recreate_collection(self):
        """
        Recreates the Qdrant collection, deleting it if it already exists.
        """
        print(f"Recreating collection '{self.collection_name}'...")
        self.client.recreate_collection(
            collection_name=self.collection_name,
            vectors_config=models.VectorParams(size=self.vector_size, distance=models.Distance.COSINE),
        )
        print(f"Collection '{self.collection_name}' recreated.")

    def upsert_vectors(self, embeddings: List[List[float]], payloads: List[Dict[str, Any]]):
        """
        Upserts vectors and their corresponding payloads into the Qdrant collection.

        Args:
            embeddings (List[List[float]]): A list of vectors (embeddings).
            payloads (List[Dict[str, Any]]): A list of dictionaries, where each dictionary
                                             is the payload for the corresponding vector.
        """
        if len(embeddings) != len(payloads):
            raise ValueError("Number of embeddings and payloads must be the same.")

        print(f"Upserting {len(embeddings)} vectors into collection '{self.collection_name}'...")
        points = []
        for i, (embedding, payload) in enumerate(zip(embeddings, payloads)):
            points.append(models.PointStruct(
                id=i,  # Qdrant will assign a UUID if not provided, but explicit IDs can be useful
                vector=embedding,
                payload=payload
            ))
        
        self.client.upsert(
            collection_name=self.collection_name,
            wait=True,
            points=points
        )
        print(f"Successfully upserted {len(embeddings)} vectors.")

    def search(self, query_embedding: List[float], limit: int = 5) -> List[Dict[str, Any]]:
        """
        Performs a vector search in the Qdrant collection.

        Args:
            query_embedding (List[float]): The embedding of the query.
            limit (int): The maximum number of results to return.

        Returns:
            List[Dict[str, Any]]: A list of search results, each containing the payload and score.
        """
        print(f"Searching collection '{self.collection_name}' for top {limit} results...")
        search_result = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=limit,
            with_payload=True
        )
        results = []
        for hit in search_result:
            results.append({"payload": hit.payload, "score": hit.score})
        print(f"Found {len(results)} results.")
        return results

if __name__ == "__main__":
    # Example usage:
    try:
        COLLECTION_NAME = "test_book_chunks"
        qdrant_service = QdrantService(collection_name=COLLECTION_NAME)

        # Ensure a collection exists
        qdrant_service.recreate_collection()

        # Dummy embeddings and payloads
        dummy_embeddings = [
            [0.1, 0.2, 0.3] * 512,  # Example: 1536-dim vector
            [0.4, 0.5, 0.6] * 512,
            [0.7, 0.8, 0.9] * 512,
        ]
        dummy_payloads = [
            {"content": "First chunk of text about robots.", "source": "ch1"},
            {"content": "Second chunk discussing AI.", "source": "ch2"},
            {"content": "Third chunk about machine learning.", "source": "ch3"},
        ]

        qdrant_service.upsert_vectors(dummy_embeddings, dummy_payloads)

        # Dummy query embedding for search
        query_embedding = [0.15, 0.25, 0.35] * 512
        search_results = qdrant_service.search(query_embedding, limit=2)

        for result in search_results:
            print(f"Score: {result['score']}, Content: {result['payload']['content']}")

    except ValueError as e:
        print(f"Configuration Error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")