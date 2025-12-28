from typing import List, Dict, Any
from backend.src.core.embedding_generator import EmbeddingGenerator
from backend.src.services.chromadb_service import ChromaDBService

class RAGRetriever:
    def __init__(self, collection_name: str):
        self.embedding_generator = EmbeddingGenerator()
        self.db_service = ChromaDBService(collection_name=collection_name)

    async def retrieve_selected_text(self, selected_text: str) -> List[Dict[str, Any]]:
        """
        Processes selected text directly for retrieval. No DB search in this mode.

        Args:
            selected_text (str): The text selected by the user.

        Returns:
            List[Dict[str, Any]]: A list containing a single dictionary representing
                                  the selected text as a chunk.
        """
        if not selected_text:
            return []
        
        return [{
            "content": selected_text,
            "source": "user_selection",
            "chunk_id": "user_selection",
            "score": 1.0
        }]

    async def retrieve_full_book(self, query: str, limit: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieves relevant document chunks from the entire book content in ChromaDB
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

        print(f"Searching ChromaDB for top {limit} relevant chunks in collection '{self.db_service.collection.name}'")
        search_results = self.db_service.query_points(query_embedding, limit=limit)

        retrieved_chunks = []
        for result in search_results:
            retrieved_chunks.append({
                "content": result["payload"].get("text"),
                "source": result["payload"].get("file_path"),
                "chunk_id": result["payload"].get("chunk_id_in_file"),
                "score": result["score"]
            })
        
        print(f"Retrieved {len(retrieved_chunks)} chunks.")
        return retrieved_chunks

if __name__ == "__main__":
    import asyncio
    
    async def test_retriever():
        COLLECTION_NAME = "book_rag_collection"
        
        try:
            retriever = RAGRetriever(collection_name=COLLECTION_NAME)
            
            sample_query = "What is a URDF file?"
            print(f"\n--- Testing full book retrieval for query: '{sample_query}' ---")
            results = await retriever.retrieve_full_book(sample_query, limit=3)
            
            if results:
                for i, chunk in enumerate(results):
                    print(f"Chunk {i+1} (Score: {chunk['score']:.2f}):")
                    print(f"  Source: {chunk['source']}")
                    print(f"  Content: {chunk['content'][:200]}...")
            else:
                print("No chunks retrieved.")

        except ValueError as e:
            print(f"Configuration Error: {e}")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")

    asyncio.run(test_retriever())
