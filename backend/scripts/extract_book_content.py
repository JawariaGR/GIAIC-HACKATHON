import os
import argparse
from pathlib import Path

def extract_content_from_markdown(file_path: Path):
    """
    Extracts text content from a markdown file.
    Placeholder for more sophisticated markdown parsing if needed.
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    # Basic cleanup: remove markdown specific syntax if desired, for now, return raw
    return content

def main():
    parser = argparse.ArgumentParser(description="Extract book content from markdown files.")
    parser.add_argument("--book_path", type=str, required=True,
                        help="Path to the root directory of the book's markdown content.")
    args = parser.parse_args()

    book_root_path = Path(args.book_path)
    if not book_root_path.is_dir():
        print(f"Error: Book path '{book_root_path}' is not a valid directory.")
        return

    extracted_data = []
    for md_file in book_root_path.rglob("*.md"):
        try:
            content = extract_content_from_markdown(md_file)
            relative_path = md_file.relative_to(book_root_path)
            extracted_data.append({
                "file_path": str(relative_path),
                "content": content
            })
            print(f"Extracted content from: {relative_path}")
        except Exception as e:
            print(f"Error processing {md_file}: {e}")

    # For now, just print the summary. Later this will pass to chunking/embedding.
    print(f"\nSuccessfully extracted content from {len(extracted_data)} markdown files.")
    # You might want to save this to a temporary JSON or similar for the next step
    # import json
    # with open("extracted_book_content.json", "w", encoding="utf-8") as f:
    #     json.dump(extracted_data, f, indent=2)

if __name__ == "__main__":
    main()
