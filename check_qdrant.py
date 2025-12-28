from qdrant_client import QdrantClient

client = QdrantClient(path='./backend/chatbot/qdrant_storage')

try:
    info = client.get_collection('physical_ai_course')
    print(f'Collection exists with {info.points_count} chunks')
except Exception as e:
    print(f'Collection not found or error: {e}')
