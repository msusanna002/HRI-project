import os
import uvicorn
from fastapi import FastAPI
from pydantic import BaseModel
from openai import OpenAI
from logger import get_logger

LOG = get_logger(__name__)
app = FastAPI()

OPENAI_KEY = os.environ.get("OPENAI_API_KEY")
client = OpenAI(api_key=OPENAI_KEY)


class Req(BaseModel):
    board_state: dict
    available: list | None = []


def parse_piece_name(name: str):
    parts = name.split("_")
    if not parts:
        return name, None

    color = parts[-1]
    shape = "_".join(parts[:-1]) if len(parts) > 1 else name
    return shape, color


@app.post("/recommend_piece")
def recommend_piece(req: Req):

    placed = req.board_state.get("placed", [])
    available = req.available or []

    LOG.info("LLM call: placed=%s available=%s", placed, available)

    if not available:
        return {"recommended_piece": None}
    #gpt-40-mini decision making
    prompt = f"""
    You are an intelligent Tangram assembly assistant for a robot.

    PLACED PIECES:
    {placed}

    AVAILABLE PIECES:
    {available}

    TASK:
    Choose one piece from AVAILABLE that best complements the already placed pieces.
    Prefer:
    - if a peice was just rejected try something different next
    - filling gaps
    Return ONLY the name of the piece."""

    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[{"role": "user", "content": prompt}],
        max_tokens=20,
        temperature=0.5
    )

    recommended = response.choices[0].message.content.strip()
    LOG.info("GPT recommended: %s", recommended)

    return {"recommended_piece": recommended}


if __name__ == "__main__":
    LOG.info("Starting llm_agent on http://127.0.0.1:8001")
    uvicorn.run("llm_agent:app", host="127.0.0.1", port=8001, reload=False)
