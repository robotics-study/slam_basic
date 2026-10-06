// 스캔 팬 모티프 로고 — 한 점에서 퍼져 나가는 빔과 궤적. robotics-study 허브의 브랜드와
// 같은 indigo→cyan 그라디언트를 쓴다. 같은 페이지에 두 개가 동시에 렌더될 수 있어
// 그라디언트 id를 인스턴스마다 달리 받는다.
const BrandLogo = ({size = 26, gradId = "navLogo"}: { size?: number; gradId?: string }) => (
    <svg className="logo" width={size} height={size} viewBox="0 0 24 24" fill="none" aria-hidden="true">
        <defs>
            <linearGradient id={gradId} x1="0" y1="1" x2="1" y2="0">
                <stop stopColor="#6366f1"/>
                <stop offset="1" stopColor="#06b6d4"/>
            </linearGradient>
        </defs>
        <path d="M5.5 18.5L19 5" stroke={`url(#${gradId})`} strokeWidth="1.6" strokeLinecap="round"
              opacity=".55" strokeDasharray="2.4 2.2"/>
        <path d="M5.5 18.5L17.5 9.5" stroke={`url(#${gradId})`} strokeWidth="1.6" strokeLinecap="round"/>
        <path d="M5.5 18.5L19 14" stroke={`url(#${gradId})`} strokeWidth="1.6" strokeLinecap="round" opacity=".9"/>
        <path d="M5.5 18.5L13 6.5" stroke={`url(#${gradId})`} strokeWidth="1.6" strokeLinecap="round" opacity=".7"/>
        <circle cx="5.5" cy="18.5" r="2.4" fill={`url(#${gradId})`}/>
    </svg>
)

export default BrandLogo
