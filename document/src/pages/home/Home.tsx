import {AlgoSection, ISupportedExample, Localized} from "../../../types/global";
import algorithms from "../algorithms";
import {ALGO_BLURBS, SECTIONS} from "../algorithms/roadmap";
import BrandLogo from "../../components/BrandLogo";
import HeroSearch from "../../components/panels/HeroSearch";
import SiteHighlights from "../../components/panels/home/SiteHighlights";
import {useAlgoNav} from "../../libs/nav";
import {useLang, useTr, pick} from "../../libs/i18n";
import cn from "../../libs/cn";

const REPO = "https://github.com/robotics-study/slam_basic"

const AlgoCard = ({slug, section, title, blurb, supportedExample, onOpen}: {
    slug: string
    section: AlgoSection
    title: Localized
    blurb?: Localized
    supportedExample?: ISupportedExample
    onOpen?: () => void
}) => {
    const {lang} = useLang()
    const t = useTr()
    const langs = supportedExample
        ? Object.entries(supportedExample).filter(([, v]) => v).map(([codeLang]) => codeLang)
        : []
    return (
        <div className={cn("doc-card", onOpen ? "clickable" : "dim")}
             role={onOpen ? "button" : undefined} tabIndex={onOpen ? 0 : undefined}
             onClick={onOpen}
             onKeyDown={(e) => onOpen && (e.key === "Enter" || e.key === " ") && onOpen()}>
            <div className="dc-head">
                <span className="dc-title">{pick(lang, title)}</span>
                {!onOpen && <span className="soon">soon</span>}
            </div>
            {blurb && <p className="dc-blurb">{pick(lang, blurb)}</p>}
            {onOpen && langs.length > 0 && (
                <div className="chips">
                    {langs.map((codeLang) => {
                        // 저장소 소스 경로 — 알고리즘 파일은 섹션 디렉토리(python/slam/<section>/ ·
                        // cpp/include/slam/<section>/)에 있다.
                        const file = codeLang === "c++"
                            ? `cpp/include/slam/${section}/${slug}.hpp`
                            : `python/slam/${section}/${slug}.py`
                        return (
                            <a key={codeLang} className="mini-chip" target="_blank" rel="noreferrer"
                               onClick={(e) => e.stopPropagation()}
                               href={`${REPO}/blob/main/${file}`}>
                                {t(`${codeLang} code`, `${codeLang} 코드`)}
                            </a>
                        )
                    })}
                </div>
            )}
        </div>
    )
}

const Home = () => {
    const {go, goSection} = useAlgoNav()
    const {lang} = useLang()
    const t = useTr()
    const ready = algorithms.filter((a) => a.contents)
    const first = ready[0]?.slug
    const blurbOf = (slug: string) => ALGO_BLURBS.find((b) => b.slug === slug)?.blurb

    return (
        <main className="lander">
            <div className="lander-top">
                <BrandLogo size={54} gradId="navLanderLogo"/>
                <h1>slam<span className="wm-dim"> study</span></h1>
                <p className="sub">
                    {t(
                        "A study of single-robot SLAM and state estimation along its genealogy — the " +
                        "recursive Bayes filter that starts everything, scan registration without " +
                        "wheels, feature extraction, filter-based SLAM from EKF to Rao-Blackwellized " +
                        "grids to tightly-coupled LiDAR-inertial odometry, and graph-based SLAM — " +
                        "each algorithm derived, proven, and shown as the real C++ and Python source " +
                        "that implements it.",
                        "계보를 따라 읽는 단일 로봇 SLAM·상태 추정 — 모든 것의 출발점인 재귀 Bayes " +
                        "필터에서, 바퀴 없는 오도메트리(스캔 등록), 특징 추출, EKF에서 Rao-Blackwellized " +
                        "격자와 긴밀 결합 라이다-관성까지 이어지는 필터 기반 SLAM, 그리고 그래프 기반 " +
                        "SLAM까지. 알고리즘마다 유도하고 증명하고, 그것을 구현한 실제 C++·Python " +
                        "소스까지 함께 읽는다.",
                    )}
                </p>
                <div className="lander-chips">
                    <span className="chip">Bayes Filter</span>
                    <span className="chip">Histogram Filter</span>
                    <span className="chip">Log-odds Grid Mapping</span>
                    <span className="chip">Particle Filter</span>
                    <span className="chip">MCL</span>
                    <span className="chip">ICP</span>
                    <span className="chip">NDT</span>
                    <span className="chip">DBSCAN</span>
                    <span className="chip">EKF-SLAM</span>
                    <span className="chip">FastSLAM</span>
                    <span className="chip">GMapping</span>
                    <span className="chip">FAST-LIO</span>
                    <span className="chip">Pose Graph · SPA</span>
                    <span className="chip">Cartographer</span>
                    <span className="chip">C++ / Python</span>
                </div>
                <div className="lander-btns">
                    {first && (
                        <button className="btn btn-primary" onClick={() => go(first)}>
                            {t("Start reading", "학습 시작")}
                        </button>
                    )}
                    <a className="btn btn-ghost" href={REPO} target="_blank" rel="noopener noreferrer">GitHub</a>
                </div>
            </div>

            <HeroSearch/>

            <SiteHighlights/>

            <div className="lander-cats">
                {SECTIONS.map((sec, si) => (
                    <div key={sec.key} className="lander-cat">
                        <div className="part-head">
                            <h3>
                                <span className="part-index">{["I", "II", "III", "IV", "V"][si]}</span>
                                {pick(lang, sec.title)}
                                <a className="part-intro" onClick={() => goSection(sec.key)}>
                                    Introduction →
                                </a>
                            </h3>
                            <p className="part-desc">{pick(lang, sec.desc)}</p>
                        </div>
                        <div className="card-grid">
                            {algorithms.filter((a) => a.section === sec.key).map((a) => (
                                <AlgoCard key={a.slug} slug={a.slug} section={a.section} title={a.title}
                                          blurb={blurbOf(a.slug)}
                                          supportedExample={a.supportedExample}
                                          onOpen={a.contents ? () => go(a.slug) : undefined}/>
                            ))}
                        </div>
                    </div>
                ))}
            </div>
        </main>
    )
}

export default Home
