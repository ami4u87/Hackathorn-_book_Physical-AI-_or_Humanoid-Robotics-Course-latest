#!/usr/bin/env python3
"""
Temporary script to load content to Qdrant Cloud
Sets environment variables to use cloud instance
"""
import os
import sys

# Set Qdrant Cloud credentials
os.environ['QDRANT_URL'] = 'https://cfc7191e-395c-4a70-ac72-979ede526d04.us-east4-0.gcp.cloud.qdrant.io'
os.environ['QDRANT_API_KEY'] = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.PtNgKDFuruWwVDasnl-xkrWShCwLnQgECdrY7Rc2n-0'

# Import and run the load_content script
sys.path.insert(0, 'scripts')
from load_content import main

if __name__ == "__main__":
    print("Loading content to Qdrant Cloud...")
    print("URL:", os.environ['QDRANT_URL'])
    print()
    main()
