import os
import chromadb
from typing import List, Dict, Any

class ChromaDBService:
    def __init__(self, collection_name: str):
        db_path = os.path.join(os.getcwd(), ".chromadb") # Use a directory in the project
        self.client = chromadb.PersistentClient(path=db_path)
        self.collection = self.client.get_or_create_collection(name=collection_name, metadata={"hnsw:space": "cosine"})

    def recreate_collection(self, collection_name: str):
        """
        Recreates the ChromaDB collection, deleting it if it already exists.
        """
        print(f"Recreating collection '{collection_name}'...")
        try:
            self.client.delete_collection(name=collection_name)
        except: # Catch all exceptions, as ChromaDB raises different errors for non-existent collections
            pass  # Collection didn't exist
        self.collection = self.client.get_or_create_collection(name=collection_name, metadata={"hnsw:space": "cosine"})
        print(f"Collection '{collection_name}' recreated.")

    def upsert_vectors(self, embeddings: List[List[float]], payloads: List[Dict[str, Any]], ids: List[str]):
        """
        Upserts vectors and their corresponding payloads into the ChromaDB collection.

        Args:
            embeddings (List[List[float]]): A list of vectors (embeddings).
            payloads (List[Dict[str, Any]]): A list of dictionaries, where each dictionary
                                             is the payload for the corresponding vector.
            ids (List[str]): A list of unique IDs for each vector.
        """
        if len(embeddings) != len(payloads) or len(embeddings) != len(ids):
            raise ValueError("Number of embeddings, payloads, and ids must be the same.")

        print(f"Upserting {len(embeddings)} vectors into collection '{self.collection.name}'...")
        self.collection.add(
            embeddings=embeddings,
            documents=[p.get("text", "") for p in payloads],
            metadatas=payloads,
            ids=ids
        )
        print(f"Successfully upserted {len(embeddings)} vectors.")

    def query_points(self, query_embedding: List[float], limit: int = 5) -> List[Dict[str, Any]]:
        """
        Performs a vector search in the ChromaDB collection.

        Args:
            query_embedding (List[float]): The embedding of the query.
            limit (int): The maximum number of results to return.

        Returns:
            List[Dict[str, Any]]: A list of search results, each containing the payload and score.
        """
        print(f"Searching collection '{self.collection.name}' for top {limit} results...")
        results = self.collection.query(
            query_embeddings=[query_embedding],
            n_results=limit
        )
        
        search_results = []
        if results and results['documents'] and len(results['documents']) > 0:
            for i, doc in enumerate(results['documents'][0]):
                search_results.append({
                    "payload": results['metadatas'][0][i],
                    "score": results['distances'][0][i]
                })

        print(f"Found {len(search_results)} results.")
        return search_results

if __name__ == "__main__":
    # Example usage:
    try:
        COLLECTION_NAME = "test_book_chunks"
        chroma_service = ChromaDBService(collection_name=COLLECTION_NAME)

        # Ensure a collection exists
        chroma_service.recreate_collection(COLLECTION_NAME)

        # Dummy embeddings and payloads
        dummy_embeddings = [
            [0.1] * 1536,
            [0.4] * 1536,
            [0.7] * 1536,
        ]
        dummy_payloads = [
            {"text": "First chunk of text about robots.", "source": "ch1"},
            {"text": "Second chunk discussing AI.", "source": "ch2"},
            {"text": "Third chunk about machine learning.", "source": "ch3"},
        ]
        dummy_ids = ["1", "2", "3"]

        chroma_service.upsert_vectors(dummy_embeddings, dummy_payloads, dummy_ids)

        # Dummy query embedding for search
        query_embedding = [0.15] * 1536
        search_results = chroma_service.query_points(query_embedding, limit=2)

        for result in search_results:
            print(f"Score: {result['score']}, Content: {result['payload']['text']}")

    except Exception as e:
        print(f"An unexpected error occurred: {e}")