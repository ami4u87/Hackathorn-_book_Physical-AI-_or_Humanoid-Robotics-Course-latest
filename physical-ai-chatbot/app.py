import gradio as gr
from huggingface_hub import InferenceClient
from sentence_transformers import SentenceTransformer
from qdrant_client import QdrantClient
import os

# Configuration
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
HF_TOKEN = os.getenv("HF_TOKEN")  # Optional: uses public inference API if not set

# Initialize Hugging Face Inference Client (free, unlimited)
client = InferenceClient(token=HF_TOKEN)
encoder = SentenceTransformer('sentence-transformers/all-MiniLM-L6-v2')

# Initialize Qdrant with better error handling and timeout
try:
    qdrant_client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
        prefer_grpc=False,
        timeout=30.0  # 30 second timeout
    )
    # Test connection
    collections = qdrant_client.get_collections()
    print(f"✅ Connected to Qdrant! Collections: {collections}")
except Exception as e:
    print(f"❌ Qdrant connection error: {e}")
    print(f"URL: {QDRANT_URL}")
    qdrant_client = None

def query_chatbot(message, history):
    """
    Process user query and return chatbot response.

    Args:
        message: Current user message
        history: Chat history (list of [user, bot] message pairs)

    Returns:
        Bot response string
    """
    # Check if Qdrant is available
    if qdrant_client is None:
        return "⚠️ Database connection unavailable. Please check the Qdrant Cloud configuration in Space settings."

    try:
        # Encode query
        query_vector = encoder.encode(message).tolist()

        # Search Qdrant with timeout handling
        try:
            search_results = qdrant_client.search(
                collection_name="physical_ai_course",
                query_vector=query_vector,
                limit=5
            )
        except Exception as search_error:
            return f"⚠️ Database search failed: {str(search_error)}\n\nPlease verify Qdrant Cloud credentials in Space settings."

        # Extract context
        context_chunks = [hit.payload.get("text", "") for hit in search_results]
        context = "\n\n".join(context_chunks)

        if not context.strip():
            return "I apologize, but I couldn't find relevant information in the course materials. Could you rephrase your question?"

        # Build prompt for chat model
        system_prompt = "You are a helpful teaching assistant for the Physical AI and Humanoid Robotics course. Provide clear, educational responses based on the provided context. Use examples when helpful and be encouraging."

        user_prompt = f"""Context from course materials:
{context}

Student question: {message}

Provide a clear, educational response based on the context above."""

        # Generate response using HF Inference API (conversational)
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt}
        ]

        response = ""
        try:
            # Use chat completion API with current supported model
            for message_chunk in client.chat_completion(
                messages=messages,
                model="meta-llama/Llama-3.2-3B-Instruct",  # Currently supported, free model
                max_tokens=500,
                temperature=0.7,
                stream=True
            ):
                if message_chunk.choices and message_chunk.choices[0].delta.content:
                    response += message_chunk.choices[0].delta.content
        except Exception as e:
            return f"⚠️ AI service error: {str(e)}\n\nPlease try again in a moment."

        return response.strip() if response else "I apologize, but I couldn't generate a response. Please try again."

    except Exception as e:
        return f"Error processing your question: {str(e)}"

# Create Gradio interface
demo = gr.ChatInterface(
    query_chatbot,
    title="🤖 Physical AI Course Chatbot",
    description="""
    Ask me anything about ROS 2, robotics, simulation, perception, and humanoid robotics!

    **Topics covered:**
    - ROS 2 fundamentals (nodes, topics, services, actions)
    - Simulation (Gazebo, Unity, Isaac Sim)
    - Perception and computer vision
    - Vision-Language-Action (VLA) models
    - Control systems and path planning
    """,
    examples=[
        "What is ROS 2?",
        "How do I create a ROS 2 publisher?",
        "Explain TF2 coordinate frames",
        "What's the difference between Gazebo and Isaac Sim?",
        "How do I integrate GPT-4 Vision with robotics?"
    ],
    theme=gr.themes.Soft(),
    retry_btn="🔄 Retry",
    undo_btn="↩️ Undo",
    clear_btn="🗑️ Clear"
)

if __name__ == "__main__":
    demo.launch()
