import argparse
import os
import json
from pathlib import Path
from typing import List, Dict, Any

# Assuming these modules are in the Python path or importable
from backend.scripts.extract_book_content import extract_content_from_markdown
from backend.src.core.text_chunker import chunk_text
from backend.src.core.embedding_generator import EmbeddingGenerator
from backend.src.services.qdrant_service import QdrantService

def prepare_data_pipeline(book_path: str, collection_name: str):
    """
    Executes the full data processing pipeline: extract, chunk, embed, and upload.

    Args:
        book_path (str): Path to the root directory of the book's markdown content.
        collection_name (str): Name of the Qdrant collection to use.
    """
    book_root_path = Path(book_path)
    if not book_root_path.is_dir():
        raise ValueError(f"Error: Book path '{book_root_path}' is not a valid directory.")

    # 1. Extract content from markdown files
    print("\n--- Step 1: Extracting book content ---")
    extracted_documents: List[Dict[str, str]] = []
    for md_file in book_root_path.rglob("*.md"):
        try:
            content = extract_content_from_markdown(md_file)
            relative_path = md_file.relative_to(book_root_path)
            extracted_documents.append({
                "file_path": str(relative_path),
                "content": content
            })
            print(f"Extracted content from: {relative_path}")
        except Exception as e:
            print(f"Error processing {md_file}: {e}")
    print(f"Total {len(extracted_documents)} documents extracted.")

    if not extracted_documents:
        print("No content extracted. Aborting data preparation.")
        return

    # 2. Chunk text
    print("\n--- Step 2: Chunking text ---")
    all_chunks: List[Dict[str, Any]] = []
    chunk_id_counter = 0
    for doc in extracted_documents:
        # Simple character-based chunking for now
        doc_chunks = chunk_text(doc["content"], chunk_size=1000, chunk_overlap=200)
        for chunk in doc_chunks:
            chunk["id"] = f"{doc['file_path']}_{chunk_id_counter}" # Unique ID for each chunk
            chunk["metadata"] = {"file_path": doc["file_path"], "original_content_length": len(doc["content"]) }
            chunk["content"] = chunk.pop("content") # Rename 'content' to 'text' if needed by Qdrant payload directly
            all_chunks.append(chunk)
            chunk_id_counter += 1
    print(f"Total {len(all_chunks)} chunks created.")

    if not all_chunks:
        print("No chunks created. Aborting data preparation.")
        return

    # 3. Generate embeddings
    print("\n--- Step 3: Generating embeddings ---")
    embedding_generator = EmbeddingGenerator()
    texts_to_embed = [chunk["content"] for chunk in all_chunks]
    embeddings = embedding_generator.generate_embeddings(texts_to_embed)
    print(f"Generated {len(embeddings)} embeddings.")

    # Prepare payloads for Qdrant
    qdrant_payloads = []
    for i, chunk in enumerate(all_chunks):
        payload = {
            "text": chunk["content"],
            "file_path": chunk["metadata"]["file_path"],
            "chunk_id_in_file": chunk["id"], # Keep a reference to the chunk ID within the file
        }
        qdrant_payloads.append(payload)

    # 4. Upload to Qdrant
    print("\n--- Step 4: Uploading vectors to Qdrant ---")
    qdrant_service = QdrantService(collection_name=collection_name)
    qdrant_service.recreate_collection() # Ensure a fresh collection for new data
    qdrant_service.upsert_vectors(embeddings, qdrant_payloads)
    print("Data preparation pipeline completed successfully.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Prepare book data for RAG chatbot.")
    parser.add_argument("--book_path", type=str, required=True,
                        help="Path to the root directory of the book's markdown content.")
    parser.add_argument("--collection_name", type=str, default="book_rag_collection",
                        help="Name of the Qdrant collection to store vectors.")
    args = parser.parse_args()

    try:
        # Adjust book_path for the project structure if the book content is in D:\Hackathon\physical_Ai_book\book
        absolute_book_path = Path(os.getcwd()) / args.book_path
        
        print(f"Starting data preparation for book path: {absolute_book_path}")
        prepare_data_pipeline(str(absolute_book_path), args.collection_name)
    except ValueError as e:
        print(f"Configuration Error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred during data preparation: {e}")