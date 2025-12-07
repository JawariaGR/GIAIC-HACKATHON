from typing import List, Dict

def chunk_text(text: str, chunk_size: int = 1000, chunk_overlap: int = 200) -> List[Dict]:
    """
    Splits a given text into smaller chunks with optional overlap.

    Args:
        text (str): The input text to be chunked.
        chunk_size (int): The desired maximum size of each text chunk (in characters).
        chunk_overlap (int): The number of characters to overlap between consecutive chunks.

    Returns:
        List[Dict]: A list of dictionaries, where each dictionary contains
                    'content' (the text chunk) and 'chunk_id' (an index).
    """
    if chunk_overlap >= chunk_size:
        raise ValueError("Chunk overlap must be less than chunk size.")

    chunks = []
    start = 0
    chunk_id = 0
    while start < len(text):
        end = min(start + chunk_size, len(text))
        chunk_content = text[start:end]
        chunks.append({"chunk_id": chunk_id, "content": chunk_content})
        
        # Move start position for the next chunk, considering overlap
        start += (chunk_size - chunk_overlap)
        if start >= len(text): # Ensure we don't create an empty last chunk if we overshot
            break
        chunk_id += 1
            
    return chunks

if __name__ == "__main__":
    sample_text = (
        "This is the first sentence of a longer paragraph. "
        "It talks about various interesting things and concepts. "
        "The second sentence continues the discussion with more details. "
        "We are making sure that the chunking utility works correctly. "
        "Here is the third sentence, adding even more information to the mix. "
        "This is the last part of the text, completing the sample for testing purposes."
    )

    print("--- Chunks with size 50, overlap 10 ---")
    small_chunks = chunk_text(sample_text, chunk_size=50, chunk_overlap=10)
    for i, chunk in enumerate(small_chunks):
        print(f"Chunk {i+1} (ID: {chunk['chunk_id']}): {chunk['content']}")

    print("\n--- Chunks with size 100, overlap 0 ---")
    no_overlap_chunks = chunk_text(sample_text, chunk_size=100, chunk_overlap=0)
    for i, chunk in enumerate(no_overlap_chunks):
        print(f"Chunk {i+1} (ID: {chunk['chunk_id']}): {chunk['content']}")

    print("\n--- Text shorter than chunk size ---")
    short_text = "A very short piece of text."
    short_text_chunks = chunk_text(short_text, chunk_size=100, chunk_overlap=0)
    for i, chunk in enumerate(short_text_chunks):
        print(f"Chunk {i+1} (ID: {chunk['chunk_id']}): {chunk['content']}")
