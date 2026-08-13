# Universal Rules

- Always answer in Chinese unless the context requires otherwise.
- Do not write defensive or fallback code; it does not solve the root problem. Prefer full exposure: let failures surface clearly (explicit errors, exceptions, logs, failing tests) so bugs are visible and can be fixed at the root cause.
- When editing existing code: if you notice unrelated dead code, mention it - don't delete it.
- Do not use Playwright MCP. When browser automation or Playwright testing is needed, use the Playwright CLI instead.
