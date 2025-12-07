import os
from typing import List
from openai import OpenAI
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

class EmbeddingGenerator:
    def __init__(self, model: str = "text-embedding-ada-002"):
        self.client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.model = model
        if not self.client.api_key:
            raise ValueError("OPENAI_API_KEY environment variable not set.")

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generates embeddings for a list of text strings using OpenAI's API.

        Args:
            texts (List[str]): A list of text strings for which to generate embeddings.

        Returns:
            List[List[float]]: A list of embeddings, where each embedding is a list of floats.
        """
        if not texts:
            return []

        all_embeddings: List[List[float]] = []
        batch_size = 16  # Recommended batch size for OpenAI embeddings

        try:
            for i in range(0, len(texts), batch_size):
                batch = texts[i:i + batch_size]
                response = self.client.embeddings.create(input=batch, model=self.model)
                all_embeddings.extend([data.embedding for data in response.data])
            return all_embeddings
        except Exception as e:
            print(f"Error generating embeddings: {e}")
            raise

if __name__ == "__main__":
    # This block will only run if the script is executed directly
    # To test, ensure OPENAI_API_KEY is set in your environment or a .env file in the backend directory.
    # Example usage:
    try:
        embedding_generator = EmbeddingGenerator()
        sample_texts = [
            "Hello, world!",
            "This is a test sentence.",
            "Another sentence for embedding."
        ]
        embeddings = embedding_generator.generate_embeddings(sample_texts)
        for i, emb in enumerate(embeddings):
            print(f"Embedding for '{sample_texts[i]}': {len(emb)} dimensions, first 5: {emb[:5]}...")
    except ValueError as e:
        print(f"Configuration Error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
