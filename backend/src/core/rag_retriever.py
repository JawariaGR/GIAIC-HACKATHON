from typing import List, Dict, Any
from backend.src.core.embedding_generator import EmbeddingGenerator
from backend.src.services.qdrant_service import QdrantService

    async def retrieve_selected_text(self, selected_text: str) -> List[Dict[str, Any]]:
        """
        Processes selected text directly for retrieval. No Qdrant search in this mode.

        Args:
            selected_text (str): The text selected by the user.

        Returns:
            List[Dict[str, Any]]: A list containing a single dictionary representing
                                  the selected text as a chunk.
        """
        if not selected_text:
            return []
        
        # In this mode, the selected text itself is the primary context.
        # We can assign a dummy source/chunk_id for tracking.
        return [{
            "content": selected_text,
            "source": "user_selection",
            "chunk_id": "user_selection",
            "score": 1.0 # Max score as it's directly provided context
        }]

    async def retrieve_full_book(self, query: str, limit: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieves relevant document chunks from the entire book content in Qdrant
        based on the user's query.

        Args:
            query (str): The user's natural language query.
            limit (int): The maximum number of relevant chunks to retrieve.

        Returns:
            List[Dict[str, Any]]: A list of dictionaries, where each dictionary represents
                                  a retrieved chunk with its content and metadata.
        """
        if not query:
            return []

        print(f"Generating embedding for query: '{query}'")
        query_embedding = self.embedding_generator.generate_embeddings([query])[0]

        print(f"Searching Qdrant for top {limit} relevant chunks in collection '{self.qdrant_service.collection_name}'")
        search_results = self.qdrant_service.search(query_embedding, limit=limit)

        retrieved_chunks = []
        for result in search_results:
            # The 'payload' contains the original text and metadata
            retrieved_chunks.append({
                "content": result["payload"].get("text"),
                "source": result["payload"].get("file_path"),
                "chunk_id": result["payload"].get("chunk_id_in_file"),
                "score": result["score"]
            })
        
        print(f"Retrieved {len(retrieved_chunks)} chunks.")
        return retrieved_chunks

if __name__ == "__main__":
    async def test_retriever():
        # Ensure QDRANT_URL, QDRANT_API_KEY, and OPENAI_API_KEY are set
        # and that the Qdrant collection 'book_rag_collection' has some data (e.g., from prepare_data.py)
        COLLECTION_NAME = "book_rag_collection" # Must match what's used in prepare_data.py
        
        try:
            retriever = RAGRetriever(qdrant_collection_name=COLLECTION_NAME)
            
            sample_query = "What is a URDF file?"
            print(f"\n--- Testing full book retrieval for query: '{sample_query}' ---")
            results = await retriever.retrieve_full_book(sample_query, limit=3)
            
            if results:
                for i, chunk in enumerate(results):
                    print(f"Chunk {i+1} (Score: {chunk['score']:.2f}):")
                    print(f"  Source: {chunk['source']}")
                    print(f"  Content: {chunk['content'][:200]}...") # Print first 200 chars
            else:
                print("No chunks retrieved.")

        except ValueError as e:
            print(f"Configuration Error: {e}")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")

    import asyncio
    asyncio.run(test_retriever())
