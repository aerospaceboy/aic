/**
 * Problem submission transport: UTF-8 text -> base64 -> fixed-size chunks.
 *
 * The backend reassembles the chunks by `id`/`i`/`n` and base64-decodes the
 * joined payload, so every chunk boundary must break the base64 text, never a
 * multi-byte UTF-8 character. Pure helpers keep this contract testable without
 * React or a ROS runtime.
 */
export const PROBLEM_CHUNK_DATA_LENGTH = 48;

export interface ProblemChunk {
  v: 1;
  id: string;
  i: number;
  n: number;
  d: string;
}

export function encodeUtf8Base64(value: string): string {
  const bytes = new TextEncoder().encode(value);
  let binary = "";
  for (const byte of bytes) {
    binary += String.fromCharCode(byte);
  }
  return btoa(binary);
}

export function splitProblemChunks(
  submissionId: string,
  value: string,
  dataLength: number = PROBLEM_CHUNK_DATA_LENGTH,
): ProblemChunk[] {
  if (!Number.isInteger(dataLength) || dataLength <= 0) {
    throw new Error("dataLength must be a positive integer");
  }
  const payload = encodeUtf8Base64(value);
  const total = Math.max(1, Math.ceil(payload.length / dataLength));
  const chunks: ProblemChunk[] = [];
  for (let index = 0; index < total; index += 1) {
    chunks.push({
      v: 1,
      id: submissionId,
      i: index,
      n: total,
      d: payload.slice(index * dataLength, (index + 1) * dataLength),
    });
  }
  return chunks;
}
